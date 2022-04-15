# ratatouille_planner

ROS package that handles high-level planning for the Ratatouille ingredient dispensing system.

## State Diagram

```mermaid
stateDiagram-v2
    [*] --> RESET
    RESET --> HOME: Open gripper
    state ingredient_selected <<choice>>
    HOME --> ingredient_selected: Is Request Pending?
    ingredient_selected --> AWAIT_USER_INPUT: False
    state dispensing_complete <<choice>>
    ingredient_selected --> dispensing_complete: True\n Is dispensing \n complete?
    dispensing_complete --> DISPENSING: False
    dispensing_complete --> REPLACE_CONTAINER: True
    AWAIT_USER_INPUT --> SEARCH_MARKER: Dispensing \n request \n received
    SEARCH_MARKER --> PICK_CONTAINER: Ingredient \n container \n found
    PICK_CONTAINER --> HOME: Pick \n ingredient \n container
    DISPENSING --> HOME: Ingredient \n dispensed
    REPLACE_CONTAINER --> HOME: Container replaced \n on shelf

    AWAIT_USER_INPUT --> ERROR: Invalid \n dispensing \n request
    SEARCH_MARKER --> ERROR: Cannot find marker
    PICK_CONTAINER --> ERROR: Unable to \n pick container
    DISPENSING --> ERROR: Unable to dispense
    REPLACE_CONTAINER --> ERROR: Unable to replace \n container on shelf
    ERROR --> RESET: Log error and \n prompt user
```

## Dependencies

- catkin
- roscpp
- rospy
- std_msgs
- ur_motion
- ratatouille_pose_estimation
- dispense
- sensor_interface
- ur5e_moveit_config
- realsense2_camera
- ar_track_alvar
- robotiq_urcap_control

## Usage

```
roslaunch ratatouille_planner dispense_autonomous.py [-h] [--debug] [--config-dir CONFIG_DIR] [--disable-gripper]
                              [--disable-external-input] [--verbose] [--stop-and-proceed]

optional arguments:
  -h, --help            show this help message and exit
  --debug               Enable debug mode (run without robot)
  --config-dir CONFIG_DIR
                        Directory path for configuration files
  --disable-gripper     Disable gripper commands
  --disable-external-input
                        Disable user input board
  --verbose             Enable verbose output
  --stop-and-proceed    Announce and wait for key-press before performing each action

```
