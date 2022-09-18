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
    has_request1 --> SEARCH_MARKER: True
    has_container --> has_request2: True \n Has Request?
    has_request2 --> DISPENSE: True
    has_request2 --> REPLACE_CONTAINER: False

    AWAIT_USER_INPUT --> HOME: has_request = True

    SEARCH_MARKER --> VERIFY_INGREDIENT
    VERIFY_INGREDIENT --> PICK_CONTAINER
    PICK_CONTAINER --> HOME: has_container = True

    REPLACE_CONTAINER --> HOME: has_container = False

    DISPENSE --> HOME: has_request = False

    LOG_ERROR --> HOME: Reset error \n has_request = False

    AWAIT_USER_INPUT --> LOG_ERROR: has_error = True
    SEARCH_MARKER --> LOG_ERROR: has_error = True
    VERIFY_INGREDIENT --> LOG_ERROR: has_error = True
    PICK_CONTAINER --> LOG_ERROR: has_error = True
    DISPENSE --> LOG_ERROR: has_error = True
    REPLACE_CONTAINER --> LOG_ERROR: has_error = True

```

## Calibration Flow State Diagram

```mermaid
stateDiagram-v2
    [*] --> HOME

    state has_request1 <<choice>>

    HOME --> has_request1: False \n Is calibration complete?

    has_request1 --> WRITE_CALIBRATION_DATA: True
    WRITE_CALIBRATION_DATA --> [*]
    has_request1 --> SEARCH_NEXT_MARKER: False

    SEARCH_NEXT_MARKER --> VERIFY_INGREDIENT
    VERIFY_INGREDIENT --> PICK_CONTAINER
    PICK_CONTAINER --> CHECK_QUANTITY

    REPLACE_CONTAINER --> HOME

    CHECK_QUANTITY --> REPLACE_CONTAINER

    LOG_ERROR --> HOME: Reset error

    
    SEARCH_NEXT_MARKER --> LOG_ERROR: has_error = True
    VERIFY_INGREDIENT --> LOG_ERROR: has_error = True
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
