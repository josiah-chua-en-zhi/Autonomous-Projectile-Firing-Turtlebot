# Autonomous-Projectile-Firing-Turtlebot

# Description

The motivation beind this project was for a module on multidsiciplinary system design, incorperating both mechenical, electical and software componenets in this project. The  objective was to design a robot that can autonomously navigate an unseeen maze, map out the lay out of the maze and stop at a loading area (for ping pong balls to be loaded in) located via an NFC tag and then search for targets with heat signatures and successfully aim and shoot a ping pong ball at it. This repository hold the codes for the navigation which was built using the ros2 middleware and the full report with the detals.

# Running 

## ts_server

This directory contains the ros2 package that should be run on the ros2 server machine.

- `wallfollow.py` Contains the main bulk of the code for the robot. It manages state to navigate the bot through the maze, manage signals from various sensors on the bot, and orchestrate the completion of the task
- `conv.py` Contain helper code to assist in transformations of odometry data into base_link, required because the tf2 python library is not fully updated to work with ros2.

### Running

On the server side, two processes must be run:
1. `ros2 launch turtlebot3_cartographer turtlebot3_cartographer.launch.py`
2. `ros2 run ts_server wallfollow`

## ts_client

This directory contains the ros2 package that should be run on the ros2 client (i.e. the turtlebot).
- `launch.py` Defines all scripts that should be launched from this package
- `button.py` Publishes Empty messages to `button` whenever the button is pressed.
- `nfc.py` Publishes Empty messages to `nfc` while a NFC tag is detected
- `thermal.py` Publishes Float32MultiArray messages to `thermal` periodically once values are read from the thermal camera.
- `motors.py` Subscribes to the `/motor` topic, in order to allow the server to control the firing of balls.


### Running

On the client side, two processes must be run:
1. `ros2 launch turtlebot3_bringup robot.launch.py`
2. `ros2 launch ~/turtlebot_ws/src/r2auto_nav/ts_client/launch.py`

## test_scripts

This directory contains standalone scripts for testing and calibrating the systems.
- `firing.py` Run to test flywheel motors, servo motor, and firing sequence.
- `calibrate_thermals.py` Run to test thermal sensor and calibrate based on the temperature of the target.

## scripts

This directory contains setup scripts for installing the code onto a new machine.
- `bootstrap_client.py` Updates packages, installs required libraries, and builds the client side code.
- `bootstrap_server.py` Installs required libraries, and builds the server side code.

# Software Architecture
![image](https://user-images.githubusercontent.com/81459293/195748615-acb7810d-e5c1-43d0-840b-5a3a16f30dcf.png)

# Electrical Architecture
## Functional Block Diagram
![image](https://user-images.githubusercontent.com/81459293/195748548-aeaa9cde-d030-4c90-a60d-78dff3026a22.png)
## Circuit Board Design
![image](https://user-images.githubusercontent.com/81459293/195753304-e1b61500-0d45-4b67-8f65-98994a4302c9.png)


