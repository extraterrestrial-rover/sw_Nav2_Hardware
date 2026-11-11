# rover_pilot

## Overview
The **`rover_pilot`** package provides control functionality for a rover platform using ROS 2.  
It manages communication between the onboard computer and the rover’s microcontroller via a serial interface.

A central script, **`ctrl.py`**, handles all serial port interactions — sending and receiving data packets to and from the microcontroller.

The package includes ROS 2 nodes that subscribe to joystick (`joy`) topics to control the rover and arm mechanisms. These nodes also log the data transmitted through the serial connection for monitoring and debugging.

## Nodes
- **`drive_node`** — Subscribes to joystick input and sends drive control commands via serial.  
- **`arm_node`** — Subscribes to joystick input and sends arm control commands via serial.  
- **`drive_node_test`** — Test version of `drive_node`.  
- **`arm_node_test`** — Test version of `arm_node`.

## Script
- **`ctrl.py`** — Core serial communication script containing functions for data transmission to the microcontroller.

## Run Syntax
```bash
ros2 run rover_pilot <node_name>

