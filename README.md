# ratatouille_planner

ROS package that handles high-level planning for the Ratatouille ingredient dispensing system.

## Dispense Flow State Diagram

```mermaid
stateDiagram-v2
    [*] --> HOME

    state has_container <<choice>>
    state has_request1 <<choice>>
    state has_request2 <<choice>>

    HOME --> has_container: Has container?

    has_container --> has_request1: False \n Has Request?
    has_request1 --> AWAIT_USER_INPUT: False
    has_request1 --> PICK_CONTAINER: True
    has_container --> has_request2: True \n Has Request?
    has_request2 --> DISPENSE: True
    has_request2 --> REPLACE_CONTAINER: False

    AWAIT_USER_INPUT --> HOME: has_request = True

    PICK_CONTAINER --> HOME: has_container = True

    REPLACE_CONTAINER --> HOME: has_container = False

    DISPENSE --> HOME: has_request = False

    LOG_ERROR --> HOME: Reset error \n has_request = False

    AWAIT_USER_INPUT --> LOG_ERROR: has_error = True
    PICK_CONTAINER --> LOG_ERROR: has_error = True
    DISPENSE --> LOG_ERROR: has_error = True
    REPLACE_CONTAINER --> LOG_ERROR: has_error = True

```

## Calibration Flow State Diagram

```mermaid
stateDiagram-v2
    [*] --> HOME
    STOP--> [*]

    state has_container <<choice>>
    state has_request1 <<choice>>
    state container_detected <<choice>>
    state ingredient_missing <<choice>>


    HOME --> has_container: Has container?
    has_container --> has_request1: False \n Is calibration complete?
    has_container --> REPLACE_CONTAINER: True

    has_request1 --> STOP: True

    has_request1 --> VISIT_NEXT_CONTAINER: False

    VISIT_NEXT_CONTAINER --> container_detected: Container detected?
    container_detected --> LABEL_INGREDIENT: True
    container_detected --> WRITE_INVENTORY_DATA: False
    LABEL_INGREDIENT --> ingredient_missing: Ingredient missing?
    ingredient_missing --> HOME: True
    ingredient_missing --> PICK_CONTAINER: False
    PICK_CONTAINER --> CHECK_QUANTITY

    REPLACE_CONTAINER --> WRITE_INVENTORY_DATA: has_container = False
    WRITE_INVENTORY_DATA --> HOME

    CHECK_QUANTITY --> HOME: has_container = True

    LOG_ERROR --> HOME: Reset error


    VISIT_NEXT_CONTAINER --> LOG_ERROR: has_error = True
    LABEL_INGREDIENT --> LOG_ERROR: has_error = True
    PICK_CONTAINER --> LOG_ERROR: has_error = True
    CHECK_QUANTITY --> LOG_ERROR: has_error = True
    REPLACE_CONTAINER --> LOG_ERROR: has_error = True

```

## Flowchart
```mermaid
flowchart
  GH[Go Home] --> HE{has_error?}
  HE --> |True| LE[Log error]
  LE --> |set has_error = False| GH

  HE --> |False| HC{has_container?}
  HC --> |False| HR1{has_request?}
  HR1 --> |False| AWAIT_USER_INPUT[Read User Input]
  AWAIT_USER_INPUT --> |Set has_request = True| GH

  HR1 --> |True| SEARCH_MARKER[Go to ingredient position & search for container]
  SEARCH_MARKER --> VF[Verify ingredient & correct pose]
  VF --> CQ[Pick container & check quantity sufficient]
  CQ --> |Set has_container = True| GH

  HC --> |True| HR2{has_request?}

  HR2 --> |True| DISPENSE[Dispense ingredient]
  DISPENSE --> |Set has_request = False| GH

  HR2 --> |False| REPLACE_CONTAINER[Replace container on shelf]
  REPLACE_CONTAINER --> |Set has_container = False| GH

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

### Inventory update
Pre-requisite services:
``` sh
roslaunch ur_motion ur5e_bringup.launch robot_ip:=10.0.0.2

roslaunch ratatouille_planner ratatouille-bringup.launch
```
Usage:
``` sh
rosrun ratatouille_planner calibrate_autonomous.py [-h] [--debug] [--config-dir CONFIG_DIR] [--disable-gripper] [--disable-external-input] [--bypass-picking]
                               [--bypass-id-service] [--bypass-sensing] [--verbose] [--stop-and-proceed]

optional arguments:
  -h, --help            show this help message and exit
  --debug               Enable debug mode (run without robot)
  --config-dir CONFIG_DIR
                        Directory path for configuration files
  --disable-gripper     Disable gripper commands
  --disable-external-input
                        Disable user input board
  --bypass-picking      Bypass container picking
  --bypass-id-service   Bypass ingredient identification
  --bypass-sensing      Bypass sensing
  --verbose             Enable verbose output
  --stop-and-proceed    Announce and wait for key-press before performing each action
```

### Dispensing
Pre-requisite services:
``` sh
roslaunch ur_motion ur5e_bringup_ratatouille.launch robot_ip:=10.0.0.2

roslaunch ratatouille_planner ratatouille-bringup.launch calibrate:=false
```
Usage:
``` sh

rosrun ratatouille_planner dispense_autonomous.py [-h] [--debug] [--config-dir CONFIG_DIR] [--dispense-log-file DISPENSE_LOG_FILE]
                              [--ingredient-quantity-log INGREDIENT_QUANTITY_LOG] [--disable-gripper] [--disable-external-input] [--bypass-dispensing]
                              [--verbose] [--stop-and-proceed]

optional arguments:
  -h, --help            show this help message and exit
  --debug               Enable debug mode (run without robot)
  --config-dir CONFIG_DIR
                        Directory path for configuration files
  --dispense-log-file DISPENSE_LOG_FILE
                        Dispensing log file path
  --ingredient-quantity-log INGREDIENT_QUANTITY_LOG
                        Ingredient quantity log file path
  --disable-gripper     Disable gripper commands
  --disable-external-input
                        Disable user input board
  --bypass-dispensing   Bypass dispensing
  --verbose             Enable verbose output
  --stop-and-proceed    Announce and wait for key-press before performing each action

```
