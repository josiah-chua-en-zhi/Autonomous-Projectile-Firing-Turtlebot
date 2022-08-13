# Python std dependencies
from collections import deque
from enum import Enum
from http.client import FOUND
import math
import cmath
import time
import traceback
import random

# ros dependencies
import rclpy
from rclpy.node import Node

# ros messaging dependencies
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, Int32, Empty

# External library dependencies
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

# Transform helper code
from .conv import *

# Update recursion limit to ensure that dfs does not fail
import sys

sys.setrecursionlimit(1000000)

# How often to update and re-execute state
UPDATE_PERIOD = 0.1

# Robot geometry information
ROBOT_FRONT = 0.14
ROBOT_LEFT = 0.12
ROBOT_RIGHT = -0.12
ROBOT_BACK = -0.09
ROBOT_LEN = ROBOT_FRONT - ROBOT_BACK

# Surroundings definition

# How far ahead to check for walls to indicate hard left turn
LOOKAHEAD = 0.10
# How far ahead to check for walls to indicate soft left turn
ADJUSTLOOKAHEAD = 0.30
# How far right to check for walls to indicate soft right turn
ADJUSTLOOKRIGHT = 0.05
# How far right to check for walls to indicate hard right turn
LOOKRIGHT = 0.20

# Wall distance to the right check
TOO_CLOSE = 0.02
TOO_FAR = 0.10

# Location of thermal camera relative to the center of the lidar sensor.
THERMAL_X = 0.09
THERMAL_Y = -0.04

# FOV of the thermal camera, should be kept constant
THERMAL_FOV = 110.0
# Width of the thermal image, should be kept constant
THERMAL_WIDTH = 32

# How many times do we have to see empty thermal readings before backing up
NOT_FOUND_THRESHOLD = 5

# How many readings do we need before trying to fire
FOUND_THRESHOLD = 50

# Which angles to consider in the thermal camera's FOV
THERMAL_ANGLE_BOUNDS = 55.0
# Range of heights in the thermal image to consider
THERMAL_H_RANGE = range(8, 16)
# Threshold temperature: must adjust!
TEMP_THRESHOLD = 40
# Which percentile of temperatures to consider
TEMP_PERCENTILE = 50

# How many balls we load
BALLS_LOADED = 3
FIRING_DIST = 0.05

# How close to target before we taper speed
TURN_TAPER_THRESHOLD = 10.0
MOVE_TAPER_THRESHOLD = 0.2

# How close to target angle before we stop
TURNING_THRESHOLD = 10
ADJUST_THRESHOLD = 30
PRECISE_THRESHOLD = 0.5

# How close to the known sighting of NFC tag do we have to be to stop
RETRACE_MARGIN = 0.05
MOVETO_MARGIN = 0.20

# How fast to go
TURNING_VEL = 0.8
ADJUST_TURNING_VEL = 0.3
PRECISE_TURNING_VEL = 0.4
FORWARD_VEL = 0.21
ADJUST_FORWARD_VEL = 0.18

MIN_FORWARD_VEL = 0.2

# Threshold to detect as wall
MAP_THRESHOLD = 60

# Minimum number of grid squares to skip, in meters squared: dependant on resolution
SKIP_THRESHOLD = 0.4

# Randomly turn in some direction after map has been completed. Enable if there are disconnected walls.
RANDOMIZE = False

# Keep on for debug purposes, else disables printing and outputting of maps.
DEBUG = True

# Keep off unless really needed for debugging. Slows down event loop significantly
DRAW = False

# Turn on to attempt to skip already seen walls before tour completion
SKIP = False

# Neighbors to check for DFS floodfill for map completion
NBORS = [(-1, 0), (1, 0), (0, 1), (0, -1)]

# State Enum
# BEGIN = the starting state, nothing happens except waiting for the lidar data and moving towards the closest wall
# SEEK = turn towards an angle, and then move forward locked until the robot meets the wall
# MOVE_TO = move towards a point, assume no obstacles in between
# FORWARD = move forwards, assume the wall is to the right
# TURN_LEFT = turn left until the wall is no longer in front
# TURN_RIGHT = turn right 90 degrees
# LOCKED_FORWARD = move forward but do not turn right (used to prevent turning right again immediately after turning right once)
# LOADING = wait for button press
# MOVE_TARGET = move towards a heat signature until a certain distance is achieved
# GATHER_TARGET = stay still and read from the thermal camera until enough data is collected
# AIM = turn around to a precise angle calculated from the target data gathered in the GATHER_TARGET state
# FIRING = repeatedly send messages to the servo to push balls until all balls have been fired
# DONE = done
State = Enum('State', 'BEGIN SEEK MOVE_TO FORWARD TURN_LEFT TURN_RIGHT LOCKED_FORWARD LOADING MOVE_TARGET GATHER_TARGET AIM FIRING DONE')

# Helper code to conver quaternion to euler rotations (in radians)
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# Determine the angle between two points on a 2D grid
def angle_to(from_x, from_y, to_x, to_y):
    return math.atan2(to_y - from_y, to_x - from_x)

# Determine the distance between two points on a 2D grid
def dist_to(from_x, from_y, to_x, to_y):
    return math.sqrt((to_y - from_y) ** 2 + (to_x - from_x) ** 2)

# Determine the clockwise rotation from one angle to another, in degrees
def angle_diff(f, t):
    return (360 + t - f) % 360

# Determine the shortest rotation from one angle to another, in degrees
def abs_angle_diff(f, t):
    return min(angle_diff(f, t), angle_diff(t, f))

# Determine if two angles are "close enough", defined by constant thresholds above
def achieved_angle(diff, precise = False):
    if precise:
        return diff < PRECISE_THRESHOLD or diff > 360.0 - PRECISE_THRESHOLD
    return diff < TURNING_THRESHOLD or diff > 360.0 - TURNING_THRESHOLD

# Return a twist that moves the bot forward and turns the bot simultaneously, slowing down the bot as it approaches a desired angle
def turn_move_towards(yaw, target, precise = False):
    twist = Twist()
    diff = angle_diff(yaw, target)
    
    if achieved_angle(diff, precise):
        return twist, True
    if (diff < ADJUST_THRESHOLD) or diff > 360 - ADJUST_THRESHOLD:
        twist.linear.x = ADJUST_FORWARD_VEL

    if(diff < 180):
        twist.angular.z = TURNING_VEL
    else:
        twist.angular.z = -TURNING_VEL
    return twist, False

# Return a twist that turns the bot, slowing down the bot as it approaches a desired angle
def turn_towards(yaw, target, precise = False):
    twist = Twist()
    diff = angle_diff(yaw, target)
    
    if achieved_angle(diff, precise):
        return twist, True
    if(diff < 180):
        twist.angular.z = taper_turn(diff, precise)
    else:
        twist.angular.z = -taper_turn(360 - diff, precise)
    return twist, False

# Helper function to taper the speed of a turn right
def scale_right_turn(delta):
    twist = Twist()
    twist.linear.x = FORWARD_VEL
    twist.angular.z = -(min(delta, 0.03) / 0.03) * TURNING_VEL
    return twist

# Helper function to taper the speed of a turn left
def scale_left_turn(delta):
    twist = Twist()
    twist.linear.x = FORWARD_VEL
    twist.angular.z = (min(delta, 0.03) / 0.03) * TURNING_VEL
    if delta >= TOO_CLOSE:
        twist.linear.x = 0.0
    return twist

# Helper function to taper the speed of a turn
def taper_turn(delta, precise = False):
    if delta > TURN_TAPER_THRESHOLD:
        if precise:
            return PRECISE_TURNING_VEL
        return TURNING_VEL
    else:
        return ADJUST_TURNING_VEL
    
# Helper function to taper the speed of a move forward
def taper_move(delta):
    if delta > MOVE_TAPER_THRESHOLD:
        return FORWARD_VEL
    else:
        return (MOVE_TAPER_THRESHOLD - delta) / MOVE_TAPER_THRESHOLD * (FORWARD_VEL - MIN_FORWARD_VEL) + MIN_FORWARD_VEL

# Helper function to convert lidar readings into a point cloud of obstacles
def generate_surroundings(laser_range):
    laser_surroundings = []
    for deg, dist in enumerate(laser_range):
        ang = np.deg2rad(deg)
        if dist != np.inf:
            rot = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
            p = np.array([dist, 0])
            p_rot = np.dot(rot, p)
            laser_surroundings.append(p_rot)
    return laser_surroundings

# Helper function to convert an angle in degrees to the index on the array returned by the thermal camera
def mlx_index(angle):
    if angle >= -THERMAL_ANGLE_BOUNDS and angle <= THERMAL_ANGLE_BOUNDS:
        index = int(round((angle + THERMAL_FOV / 2) / THERMAL_FOV * THERMAL_WIDTH))
        if index >= 0 and index < THERMAL_WIDTH:
            return index
    return None

class AutoNav(Node):
    def __init__(self):
        super().__init__('auto_nav')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        self.motor_publisher_ = self.create_publisher(Int32,'motor',10)
        

        # Set up subscriptions to needed topics
        self._odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self._occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        
        self._scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)

        self._therm_subscription = self.create_subscription(
            Float32MultiArray,
            'thermal',
            self.thermal_callback,
            qos_profile_sensor_data)
        
        self._nfc_subscription = self.create_subscription(
            Empty,
            'nfc',
            self.nfc_callback,
            qos_profile_sensor_data)
        
        self._button_subscription = self.create_subscription(
            Empty,
            'button',
            self.button_callback,
            qos_profile_sensor_data)

        self._timer = self.create_timer(UPDATE_PERIOD, self.timer_callback)

        self.tfBuffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=60))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        # Array of laser ranges returned by lidar
        self.laser_range = None
        # Point cloud generated by lidar
        self.laser_surroundings = None
        # Raw msg sent by lidar publisher
        self.scan_data = None

        # Point cloud of locations that have detected heat signatures
        self.thermal_surroundings = None

        # Occupancy map
        self.occdata = None
        # Occupancy map with a rear raycast included, which only checks for map completion behind the bot (only used for skipping algorithm)
        self.raycast = None
        # Amount of area behind the bot
        self.area = 0

        # Occupancy map metadata
        self.map_info = None

        # Odometry data returned by odom publisher
        self.odom = None
        # Base link calculated by tf2 listener
        self.base_link = None
        
        # x, y, and yaw in m and degrees, calculated from base_link.
        self.x = None
        self.y = None
        self.yaw = None

        # Current motor state, last published
        self.current_twist = None

        # Thermal data returned by thermal publisher
        self.thermal = None
        
        # If the bot has stopped at an NFC, it will be set to true.
        self.seen_nfc = False
        # Whether or not the bot has finished a full round around the maze.
        # Manually set to True to test firing only.
        self.toured = False
        
        # Known NFC location, if it has been detected before the tour is completed
        self.nfc_loc = None
        # Number of occupancy maps seen so far. Do not attempt to detect for tour completion until this reaches a certain threshold, because early maps are inaccurate.
        
        self.maps_seen = 0
        # Number of balls loaded.
        # Set to BALLS_LOADED to test firing only.
        self.loaded = 0

        # Initial state
        self.state = State.BEGIN
        self.state_data = {}

        # If a live map should be drawn, then set up matplotlib.
        if DRAW:
            # Plot config
            plt.ion()
            self.f, self.axs = plt.subplots(2, 2)
            # plt.gca().invert_yaxis()

            plt.show()

    # Helper function to determine if the bot is in a targeting state. If so, no interruptions should be made.
    def is_target_state(self):
        return (self.state == State.AIM) or (self.state == State.FIRING) or (self.state == State.GATHER_TARGET) or (self.state == State.MOVE_TARGET)

    # Get the current location of the bot.
    def current_location(self):
        start = time.time_ns()
        if self.odom is None:
            return None, None, None
        odom_pose = PoseStamped(
            header=self.odom.header,
            pose=self.odom.pose.pose,
        )

        last_error = None
        # Because the odom -> map transform may not be up to date due to latency, check older timing until we find a suitable transform. 
        for _tries in range(20):
            try:
                base_link = self.tfBuffer.transform(odom_pose, 'map')
            except Exception as e:
                last_error = e
                if odom_pose.header.stamp.nanosec >= 100000000:
                    odom_pose.header.stamp.nanosec -= 100000000
                else:
                    odom_pose.header.stamp.sec -= 1
                    odom_pose.header.stamp.nanosec += 900000000
                if e is KeyboardInterrupt:
                    raise e
            else:
                break
        else:
            print("failed to get location:", last_error)
            return None, None, None
        _roll, _pitch, yaw = euler_from_quaternion(base_link.pose.orientation.x, base_link.pose.orientation.y, base_link.pose.orientation.z, base_link.pose.orientation.w)
        
        ms = (time.time_ns() - start) / 1000000
        if ms > 100:
            print(f"found time in {ms} ms")
        return base_link.pose.position.x, base_link.pose.position.y, np.rad2deg(yaw)

    # Check the front for obstacles
    def front(self):
        backmost = math.inf
        for x, y in self.laser_surroundings:
            if x >= ROBOT_FRONT and y <= ROBOT_LEFT and y >= ROBOT_RIGHT:
                backmost = min(x, backmost)
        return backmost - ROBOT_FRONT

    # Check the front right for obstacles
    def front_right_half(self):
        backmost = math.inf
        for x, y in self.laser_surroundings:
            if x >= ROBOT_FRONT and y <= 0 and y >= ROBOT_RIGHT - ADJUSTLOOKRIGHT:
                backmost = min(x, backmost)
        return backmost - ROBOT_FRONT
    
    # Check the right for obstacles
    def right(self):
        leftmost = -math.inf
        for x, y in self.laser_surroundings:
            if x <= ROBOT_FRONT and x >= ROBOT_BACK and y <= ROBOT_RIGHT:
                leftmost = max(y, leftmost)
        return ROBOT_RIGHT - leftmost
    
    # Convert a coordinate given in meters relative to the map origin, into indexes for the occupancy map
    def to_map_coords(self, x, y, yaw):
        map_position = self.map_info.origin.position
        map_orientation = self.map_info.origin.orientation

        map_rotation = Rotation.from_quat([map_orientation.x, map_orientation.y, map_orientation.z, map_orientation.w])
        offset_x = x - map_position.x
        offset_y = y - map_position.y

        rotated_offset = map_rotation.inv().apply(np.array([offset_x, offset_y, 0]))
        map_x = int(rotated_offset[1] // self.map_info.resolution)
        map_y = int(rotated_offset[0] // self.map_info.resolution)
        _, _, offset_yaw = euler_from_quaternion(map_orientation.x, map_orientation.y, map_orientation.z, map_orientation.w)


        return map_x, map_y, yaw - offset_yaw

    # Inverse function of to_map_coords, convert a given map index into real coordinates
    def to_real_coords(self, x, y, yaw):
        map_position = self.map_info.origin.position
        map_orientation = self.map_info.origin.orientation
        map_rotation = Rotation.from_quat([map_orientation.x, map_orientation.y, map_orientation.z, map_orientation.w])

        rotated_offset = [y * self.map_info.resolution, x * self.map_info.resolution, 0]

        offsets = map_rotation.apply(rotated_offset)

        real_x = offsets[0] + map_position.x
        real_y = offsets[1] + map_position.y

        _, _, offset_yaw = euler_from_quaternion(map_orientation.x, map_orientation.y, map_orientation.z, map_orientation.w)

        return real_x, real_y, yaw + offset_yaw

    # Check if an index is within the map
    def valid_point(self, x, y):
        return x >= 0 and x < np.size(self.occdata, 0) and y >= 0 and y < np.size(self.occdata, 1)

    # DFS and attempt to reach the edge by only moving through traversable or unknown grid squares.
    # If we can reach the edge, then the map is incomplete.
    def floodfill_edges(self, x, y):
        if x == 0 or x == np.size(self.occdata, 0) - 1 or y == 0 or y == np.size(self.occdata, 1):
            return False
        for dx, dy in NBORS:
            cx = dx + x
            cy = dy + y
            if self.valid_point(cx, cy) and self.floodfill_vis[cx][cy] == 0:
                self.floodfill_vis[cx][cy] = 1
                if self.occdata[cx][cy] < MAP_THRESHOLD:
                    if not self.floodfill_edges(cx, cy):
                        return False
        return True

    # Draw a line on the map behind the bot, perpendicular to its current heading.
    def generate_raycast(self, map_x, map_y, map_yaw):
        raycast = np.full_like(self.occdata, 0, dtype=np.int32)

        slope = -1 / math.tan(np.deg2rad(map_yaw))

        head_1 = None
        head_2 = None

        dx = 0
        while True:
            cx = map_x + dx
            bound_1 = round(slope * dx + map_y)
            bound_2 = round(slope * (dx + 1) + map_y)
            to_mark = range(min(bound_1, bound_2), max(bound_1, bound_2) + 1)
            if slope < 0:
                to_mark = reversed(to_mark)
            for cy in to_mark:
                if not self.valid_point(cx, cy) or self.occdata[cx][cy] >= MAP_THRESHOLD:
                    head_1 = [cx, cy]
                    break
                raycast[cx][cy] = 1
            else:
                dx += 1
                continue
            break
        
        dx = -1
        while True:
            cx = map_x + dx
            bound_1 = round(slope * dx + map_y)
            bound_2 = round(slope * (dx + 1) + map_y)
            to_mark = range(min(bound_1, bound_2), max(bound_1, bound_2) + 1)
            if slope > 0:
                to_mark = reversed(to_mark)
            for cy in to_mark:
                if not self.valid_point(cx, cy) or self.occdata[cx][cy] >= MAP_THRESHOLD:
                    head_2 = [cx, cy]
                    break
                
                raycast[cx][cy] = 1
            else:
                dx -= 1
                continue
            break
        
        return raycast, head_1, head_2

    # After blocking off a line of squares behind the bot, check if we can still reach the edge.
    # If not, that means that everything in front has been explored.
    # Therefore, we can make a 90 degree turn left to skip past the section.
    def floodfill_infront(self, map_x, map_y):
        total = 1
        if map_x == 0 or map_x == np.size(self.occdata, 0) - 1 or map_y == 0 or map_y == np.size(self.occdata, 1):
            return False, 0
        for dx, dy in NBORS:
            cx = dx + map_x
            cy = dy + map_y
            if self.valid_point(cx, cy) and self.floodfill_vis[cx][cy] == 0 and self.raycast[cx][cy] == 0:
                self.floodfill_vis[cx][cy] = 1
                if self.occdata[cx][cy] < MAP_THRESHOLD:
                    enclosed, new_total = self.floodfill_infront(cx, cy)
                    if not enclosed:
                        return False, 0
                    total += new_total
        return True, total

    # Update location on new odom data
    def odom_callback(self, msg):
        self.odom = msg
        self.x, self.y, self.yaw = self.current_location()
        self.update_state_odom()
        self.execute_state()

    def occ_callback(self, msg):
        msgdata = np.array(msg.data)
        oc2 = msgdata
        self.occdata = oc2.reshape(msg.info.height,msg.info.width)
        self.map_info = msg.info
        self.maps_seen += 1

        # Every 5 maps, check if the tour has been completed (this takes about ~0.1 seconds, which will cause latency issues if run too frequently)
        if self.maps_seen % 5 == 0 and not self.toured:
            if self.x is None:
                return
            map_x, map_y, map_yaw = self.to_map_coords(self.x, self.y, self.yaw)
            self.floodfill_vis = np.full_like(self.occdata, 0, dtype=np.int32)
            self.toured = self.floodfill_edges(map_x, map_y)
            if DRAW:
                self.axs[1][0].imshow(self.floodfill_vis, origin='lower')
                self.f.canvas.draw()
                self.f.canvas.flush_events()

            if self.toured:
                print("Tour complete!")

            self.update_state_skip()
        if DRAW:
            self.axs[0][0].imshow(self.occdata, origin='lower')
            self.f.canvas.draw()
            self.f.canvas.flush_events()
        self.execute_state()

    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges)
        self.laser_range[self.laser_range==0] = np.inf
        self.scan_data = msg
        # Generate point cloud for use to locate obstacles and target
        self.laser_surroundings = generate_surroundings(self.laser_range)

        self.update_state_scan()
        self.execute_state()

    def thermal_callback(self, msg):
        msgdata = np.array(msg.data)
        self.thermal = msgdata.reshape([24,32])
        self.thermal_surroundings = []
        if self.laser_surroundings is not None:
            # Generate thermal point cloud
            for x, y in self.laser_surroundings:
                dx = x - THERMAL_X
                dy = y - THERMAL_Y

                # Swap x and y because we are measuring angle with respect to the y axis
                ang = np.rad2deg(math.atan2(dy, dx))
                index = mlx_index(ang)
                if index is None:
                    continue
                temps = []
                for h in THERMAL_H_RANGE:
                    temps.append(self.thermal[h][index])
                temp_per = np.percentile(temps, TEMP_PERCENTILE)
                if temp_per >= TEMP_THRESHOLD:
                    self.thermal_surroundings.append((x, y, temp_per))
        self.update_state_thermal()

    def nfc_callback(self, _msg):
        self.update_state_nfc()

    def button_callback(self, _msg):
        self.update_state_button()

    def timer_callback(self):
        self.execute_state()

    # Function to change state. New state is logged, and any extra data that the state needs to operate is generated
    def change_state(self, new_state, **kwargs):
        if self.x is None:
            self.stopbot()
            return

        if new_state == State.SEEK:
            self.state_data = { "target_angle": (self.yaw + kwargs["target_angle"]) % 360, "precise": kwargs.get("precise", False) }
        elif new_state == State.TURN_RIGHT:
            self.state_data = { "start_angle": self.yaw }
        elif new_state == State.LOCKED_FORWARD:
            self.state_data = { "dist": kwargs.get("dist"), "start": (self.x, self.y) }
        elif new_state == State.LOADING:
            self.state_data = {
                "previous_state": self.state,
                "previous_state_data": self.state_data,
            }
        elif new_state == State.MOVE_TO:
            self.state_data = { "head_x": kwargs["head_x"], "head_y": kwargs["head_y"], "start": (self.x, self.y), "achieved": False }
        elif new_state == State.MOVE_TARGET:
            self.state_data = { "head_x": kwargs["head_x"], "head_y": kwargs["head_y"] }
        elif new_state == State.GATHER_TARGET:
            self.state_data = { "collected_points": [], "not_found": 0 }
        elif new_state == State.AIM:
            self.state_data = { "head_x": kwargs["head_x"], "head_y": kwargs["head_y"] }
        else:
            self.state_data = {}
        self.state = new_state
        print("state", self.state, self.state_data)
        self.execute_state()

    # Function to update state to State.MOVE_TO, with the target point being the detected wall on the robot's left.
    # This occurs if everything in front has been detected to be explored.
    def update_state_skip(self):
        if self.x is None:
            return
        if SKIP and not self.toured and (self.state == State.FORWARD or self.state == State.TURN_LEFT or self.state == State.TURN_RIGHT or self.state == State.MOVE_TO):
            max_yaw = None
            correct_head_x = None
            correct_head_y = None
            # Increase range size to increase aggressiveness of skipping. Testing shows that values above 30 cause issues with the bot turning around in a loop.
            for i in range(1):
                try_yaw = (self.yaw + i) % 360

                map_x, map_y, map_yaw = self.to_map_coords(self.x, self.y, try_yaw)
                self.floodfill_vis = np.full_like(self.occdata, 0, dtype=np.int32)
                
                yaw_rad = np.deg2rad(try_yaw)
                rot = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad)], [math.sin(yaw_rad), math.cos(yaw_rad)]])
                
                p = np.array([ROBOT_BACK, 0])
                #p = np.array([0, 0])
                trans = np.dot(rot, p)

                back_x = self.x + trans[0]
                back_y = self.y + trans[1]
                
                map_back_x, map_back_y, _ = self.to_map_coords(back_x, back_y, try_yaw)
                if not self.valid_point(map_back_x, map_back_y):
                    break
                self.raycast, head_1, head_2 = self.generate_raycast(map_back_x, map_back_y, try_yaw)

                real_head_1_x, real_head_1_y, _ = self.to_real_coords(*head_1, 0)
                if np.cross(np.array([real_head_1_x, real_head_1_y]) - np.array([self.x, self.y]), -trans) < 0:
                    head_x, head_y = head_1
                else:
                    head_x, head_y = head_2                
                start_point = [map_back_x, map_back_y]
                
                p = np.array([0.02, 0])
                trans = np.dot(rot, p)

                cx = self.x
                cy = self.y

                while start_point == (map_x, map_y) or self.raycast[start_point[0]][start_point[1]] == 1:
                    cx += trans[0]
                    cy += trans[1]
                    if not self.valid_point(cx, cy):
                        return
                    start_point[0], start_point[1], _ = self.to_map_coords(cx, cy, try_yaw)

                enclosed, count = self.floodfill_infront(*start_point)
                if enclosed:
                    if count * self.map_info.resolution ** 2 >= SKIP_THRESHOLD:
                        max_yaw = i
                        correct_head_x, correct_head_y, _ = self.to_real_coords(head_x, head_y, 0)
                        self.change_state(State.MOVE_TO, head_x = correct_head_x, head_y = correct_head_y)
                        print("Already explored, skipping")
                    
                        for x in range(np.size(self.raycast, 0)):
                                for y in range(np.size(self.raycast, 1)):
                                    if self.raycast[x][y] == 1:
                                        self.floodfill_vis[x][y] += 2
                        self.floodfill_vis[map_x][map_y] = 4
                        if self.valid_point(start_point[0], start_point[1]):
                            self.floodfill_vis[start_point[0]][start_point[1]] = 5
                        if self.valid_point(head_x, head_y):
                            self.floodfill_vis[head_x][head_y] = 8
                        if DRAW:
                            self.axs[1][1].imshow(self.floodfill_vis, origin='lower')
                            self.f.canvas.draw()
                            self.f.canvas.flush_events()

                else:
                    break
       
    # Update state if NFC detected for the first time after tour completion
    def update_state_nfc(self):
        if self.toured:
            if not self.seen_nfc:
                self.seen_nfc = True
                self.change_state(State.LOADING)
                self.stopbot()
        elif self.nfc_loc is None:
            print("Found NFC, but waiting for tour completion first.")
            if self.x is None:
                return
            self.nfc_loc = (self.x, self.y)

    # Update state if button pressed while loading NFC
    def update_state_button(self):
        if self.state == State.LOADING:
            print("Button pressed, resuming.")
            self.state = self.state_data["previous_state"]
            self.state_data = self.state_data["previous_state_data"]
            self.loaded = BALLS_LOADED

    # Multiple different situations to update state when lidar scan is recieved
    def update_state_scan(self):
        if self.state == State.BEGIN:
            if self.laser_range is not None:
                self.change_state(State.SEEK, target_angle = np.nanargmin(self.laser_range))
        # If moving forward, turn right if there is no wall to the right, and turn left if there is a wall in front.
        elif self.state == State.FORWARD:
            if self.right() > LOOKRIGHT:
                self.change_state(State.TURN_RIGHT)
            elif self.front() < LOOKAHEAD:
                self.change_state(State.TURN_LEFT)
        # If turning left, move forward once there is no longer a wall in front.
        elif self.state == State.TURN_LEFT:
            if not self.front() < LOOKAHEAD:
                self.change_state(State.FORWARD)
        #elif self.state == State.TURN_RIGHT:
        #    if not self.right() > LOOKRIGHT:
        #        self.change_state(State.FORWARD)
        
        # If locked forward, right turns are forbidden, only left turns allowed.
        elif self.state == State.LOCKED_FORWARD:
            if self.front() < LOOKAHEAD:
                self.change_state(State.TURN_LEFT)
        
        # If moving towards a point, and close enought to target point, exit and return to either forward or turn left depending on surroundings
        elif self.state == State.MOVE_TO:
            if self.x is None:
                return
            if self.state_data["achieved"]:
                if self.right() < LOOKRIGHT and dist_to(self.x, self.y, self.state_data["head_x"], self.state_data["head_y"]) >= ROBOT_LEN:
                    self.change_state(State.FORWARD)
                if self.front() < LOOKAHEAD:
                    self.change_state(State.TURN_LEFT)
        
        # If aiming and the correct angle is achieved, start firing
        elif self.state == State.AIM:
            if self.x is None:
                return
            _, achieved = turn_towards(self.yaw, np.rad2deg(angle_to(self.x, self.y, self.state_data["head_x"], self.state_data["head_y"])), True)
            if achieved:
                self.change_state(State.FIRING)
                
        # If the maze walls are disconnected, we need to occasionally make a random left turn to attempt to explore the full maze
        if RANDOMIZE and self.toured and self.seen_nfc:
            if random.randint(0, 50)  == 0:
                self.change_state(State.SEEK, target_angle=90.0)

    # Multiple different situations to update state when new location data is recieved
    def update_state_odom(self):
        if self.x is None:
            self.stopbot()
            return
        # If turning towards a wall and the correct angle is achieved, start moving forward
        if self.state == State.SEEK:
            _, achieved = turn_towards(self.yaw, self.state_data["target_angle"], self.state_data["precise"])
            if achieved:
                self.change_state(State.LOCKED_FORWARD)
        # If turning right and 90 degrees has passed, start moving forward
        elif self.state == State.TURN_RIGHT:
            if min((self.yaw - self.state_data["start_angle"] + 360) % 360, 360 - (self.yaw - self.state_data["start_angle"] + 360) % 360) >= 90:
                self.change_state(State.LOCKED_FORWARD, dist=ROBOT_LEN)
        # If locked forward and the required distance has passed, change to just moving forward
        elif self.state == State.LOCKED_FORWARD:
            if self.state_data["dist"] is not None and dist_to(*self.state_data["start"], self.x, self.y) >= self.state_data["dist"]:
                self.change_state(State.FORWARD)
        # If moving towards a target and the correct angle is achieved, stop and scan the target for more precise readings
        elif self.state == State.MOVE_TARGET:
            _, achieved = turn_towards(self.yaw, np.rad2deg(angle_to(self.x, self.y, self.state_data["head_x"], self.state_data["head_y"])), True)
            if achieved:
                if self.front() < FIRING_DIST:
                    self.change_state(State.GATHER_TARGET)

        if self.toured and self.nfc_loc is not None and not self.seen_nfc and dist_to(self.x, self.y, self.nfc_loc[0], self.nfc_loc[1]) < RETRACE_MARGIN:
            self.seen_nfc = True
            self.change_state(State.LOADING)
            self.stopbot()
            

    def update_state_thermal(self):
        # If currently trying to scan the target, take readings until either we see that the target is actually not present, or we reach a threshold
        # Then, turn towards the average of the points.
        if self.state == State.GATHER_TARGET:
            self.state_data["collected_points"] += self.thermal_surroundings
            if len(self.state_data["collected_points"]) >= FOUND_THRESHOLD:
                if self.x is None:
                    return
                sx = 0.0
                sy = 0.0
                for xt, yt, temp in self.state_data["collected_points"]:
                    sx += xt
                    sy += yt

                ax = sx / len(self.state_data["collected_points"])
                ay = sy / len(self.state_data["collected_points"])
                print("Firing at", ax, ay)

                yaw_rad = np.deg2rad(self.yaw)
                rot = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad)], [math.sin(yaw_rad), math.cos(yaw_rad)]])

                head_x, head_y = np.dot(rot, [ax, ay]) + np.array([self.x, self.y])

                self.change_state(State.AIM, head_x = head_x, head_y = head_y)
            if len(self.thermal_surroundings) == 0:
                self.state_data["not_found"] += 1
            else:
                self.state_data["not_found"] = 0
            if self.state_data["not_found"] >= NOT_FOUND_THRESHOLD:
                self.change_state(State.SEEK, target_angle=180.0)

        if len(self.thermal_surroundings) != 0:
            # Calculate average location of hot points
            sx = 0.0
            sy = 0.0
            for xt, yt, temp in self.thermal_surroundings:
                sx += xt
                sy += yt
                
            ax = sx / len(self.thermal_surroundings)
            ay = sy / len(self.thermal_surroundings)

            print("Found heat at:", ax, ay)

            # If we see heat and are loaded, move towards the target
            if len(self.thermal_surroundings) >= 5 and self.loaded != 0 and not self.is_target_state():
                if self.toured:
                    if self.x is None:
                        return
                    yaw_rad = np.deg2rad(self.yaw)
                    rot = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad)], [math.sin(yaw_rad), math.cos(yaw_rad)]])

                    head_x, head_y = np.dot(rot, [ax, ay]) + np.array([self.x, self.y])
                    self.change_state(State.MOVE_TARGET, head_x = head_x, head_y = head_y)
                else:
                    print("Found target, but waiting for tour completion first")

    # How each state translates into actions of the robot        
    def execute_state(self):
        if self.x is None:
            self.stopbot()
            return
        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x = 0.0

        if self.state == State.SEEK:
            twist, _ = turn_towards(self.yaw, self.state_data["target_angle"], self.state_data["precise"])
        elif self.state == State.FORWARD:
            twist.linear.x = ADJUST_FORWARD_VEL
            twist.angular.z = -ADJUST_TURNING_VEL
            if self.front_right_half() < ADJUSTLOOKAHEAD:
                twist.angular.z = ADJUST_TURNING_VEL
        elif self.state == State.TURN_LEFT:
            twist.angular.z = TURNING_VEL
        elif self.state == State.TURN_RIGHT:
            twist.angular.z = -TURNING_VEL
        elif self.state == State.LOCKED_FORWARD:
            twist.linear.x = FORWARD_VEL
        elif self.state == State.MOVE_TARGET:
            twist, achieved = turn_towards(self.yaw, np.rad2deg(angle_to(self.x, self.y, self.state_data["head_x"], self.state_data["head_y"])), True)
            if achieved:
                twist.angular.z = 0.0
                twist.linear.x = FORWARD_VEL
        elif self.state == State.MOVE_TO:
            twist, achieved = turn_move_towards(self.yaw, np.rad2deg(angle_to(self.x, self.y, self.state_data["head_x"], self.state_data["head_y"])), False)
            if achieved:
                self.state_data["achieved"] = True
                twist.angular.z = 0.0
                twist.linear.x = FORWARD_VEL
        elif self.state == State.AIM:
            twist, _ = turn_towards(self.yaw, np.rad2deg(angle_to(self.x, self.y, self.state_data["head_x"], self.state_data["head_y"])), True)
        elif self.state == State.FIRING:
            self.stopbot()
            time.sleep(2)
            self.motor_publisher_.publish(Int32(data=1))
            time.sleep(10)
            while self.loaded > 0:
                self.motor_publisher_.publish(Int32(data=2))
                time.sleep(1)
                self.loaded -= 1
            self.motor_publisher_.publish(Int32(data=0))
            self.change_state(State.DONE)

        if self.current_twist != twist:
            self.publisher_.publish(twist)
            self.current_twist = twist

    def mover(self):
        try:
            while rclpy.ok():              
                rclpy.spin(self)
        except Exception as e:
            traceback.print_exc()
        finally:
            self.stopbot()

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    auto_nav = AutoNav()
    auto_nav.mover()

    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
