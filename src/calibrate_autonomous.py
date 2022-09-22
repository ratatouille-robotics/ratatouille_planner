#!/usr/bin/env python3

import sys
import rospy
import rospkg
import argparse
import yaml
import os
from enum import Enum, auto
from tf.transformations import *
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, String
from typing import Dict, List
import time
from datetime import datetime

from motion.commander import RobotMoveGroup
from ratatouille_pose_transforms.transforms import PoseTransforms
from motion.utils import make_pose, offset_pose, offset_pose_relative
from dispense.dispense import Dispenser
from sensor_interface.msg import UserInput
from ingredient_validation.srv import ValidateIngredient

_CALIBRATION_START_CONTAINER = 1
_CALIBRATION_END_CONTAINER = 15

_ROS_RATE = 10.0
_ROS_NODE_NAME = "ratatouille_planner"
_DISPENSE_THRESHOLD = 50
_INGREDIENT_DETECTION_TIMEOUT_SECONDS = 5
_MARKER_DETECTION_TIMEOUT_SECONDS = 5
_OFFSET_CONTAINER_VIEW = [0.00, 0.20, -0.07]
_CONTAINER_LIFT_OFFSET = 0.015
_CONTAINER_SHELF_BACKOUT_OFFSET = 0.20
_INVENTORY_FILE_PATH = "inventory.yaml"

# ASSUMPTIONS
# - all markers should be at correct positions (eg. marker 1 at position 1)
# - all containers should be present, no empty positions without container
# - all containers are of same height, limited by UR5e arm reach (software fully supports it)


class IngredientType(Enum):
    SALT = (auto(),)
    SUGAR = (auto(),)
    NO_INGREDIENT = auto()
    NO_CONTAINER = auto()


class Container(yaml.YAMLObject):
    yaml_tag = "!container"
    name: str = None
    quantity: float = None

    def __init__(self, _name: str, _quantity: float):
        self.name = _name
        self.quantity = _quantity

    def __repr__(self) -> str:
        return f"name: {self.name}, quantity: {self.quantity}"


yaml.add_path_resolver("!container", ["Container"], dict)


class Shelf(yaml.YAMLObject):
    yaml_tag = "!shelf"
    positions: Dict[int, Container] = None

    def __init__(self):
        self.positions = {
            key: None
            for key in range(
                _CALIBRATION_START_CONTAINER, _CALIBRATION_END_CONTAINER + 1
            )
        }


yaml.add_path_resolver("!shelf", ["Shelf"], dict)


class RatatouilleStates(Enum):
    HOME = auto()
    WRITE_CALIBRATION_DATA = auto()
    VISIT_NEXT_CONTAINER = auto()
    LABEL_INGREDIENT = auto()
    PICK_CONTAINER = auto()
    CHECK_QUANTITY = auto()
    REPLACE_CONTAINER = auto()
    LOG_ERROR = auto()
    STOP = auto()


class CalibrationStateMachine:
    def __init__(self):
        self.state = RatatouilleStates.HOME


class Ratatouille:
    # flags
    debug_mode = None
    disable_gripper = None
    verbose = None
    stop_and_proceed = None
    disable_external_input = None

    # dependencies
    robot_mg = None
    known_poses = None
    pose_transformer = None
    dispensing_update_publisher = None

    # state variables
    state: RatatouilleStates = None
    error_message: str = None
    ingredient_quantities: dict = None
    calibration_data = Shelf()

    # log files
    dispense_log_file: str = None

    def __init__(
        self,
        state: RatatouilleStates,
        config_dir_path: str,
        dispense_log_file: str,
        ingredient_quantity_log: str,
        disable_gripper: bool = False,
        verbose: bool = False,
        stop_and_proceed: bool = False,
        debug_mode: bool = False,
        disable_external_input: bool = False,
    ) -> None:
        self.config_dir_path = config_dir_path

        # initialize flags
        self.debug_mode = debug_mode
        self.disable_gripper = disable_gripper
        self.verbose = verbose
        self.stop_and_proceed = stop_and_proceed
        self.disable_external_input = disable_external_input

        with open(
            file=os.path.join(self.config_dir_path, _INVENTORY_FILE_PATH),
            mode="r",
        ) as _temp:
            _inventory = yaml.load(_temp)
            print(f"Inventory: {_inventory}")
            for key in _inventory:
                self.calibration_data.positions[key] = _inventory[key]

        # initialize dependencies
        if not self.debug_mode:
            self.robot_mg = RobotMoveGroup()

        with open(
            file=os.path.join(config_dir_path, "ingredient_quantities.yaml"),
            mode="r",
        ) as _temp:
            self.ingredient_quantities = yaml.safe_load(_temp)

        with open(
            file=os.path.join(config_dir_path, "poses.yaml"),
            mode="r",
        ) as _temp:
            self.known_poses = yaml.safe_load(_temp)
            self.known_poses["cartesian"]["ingredients"] = sorted(
                self.known_poses["cartesian"]["ingredients"], key=lambda x: x["id"]
            )
        self.pouring_characteristics = {}
        for ingredient in self.known_poses["cartesian"]["ingredients"]:
            with open(
                file=os.path.join(
                    config_dir_path, "ingredient_params", ingredient["name"] + ".yaml"
                ),
                mode="r",
            ) as f:
                ingredient_params = yaml.safe_load(f)
                self.pouring_characteristics[ingredient["name"]] = ingredient_params

        self.pose_transformer = PoseTransforms()
        self.dispensing_update_publisher = rospy.Publisher(
            "dispensing_update", String, queue_size=1
        )

        # initialize state variables
        self.state = state
        self.has_container = False
        self.ingredient_id = None
        self.ingredient_position = None
        self.ingredient_name = None
        self.error_message = None

        # initialize log file paths
        self.dispense_log_file = dispense_log_file
        self.ingredient_quantity_log = ingredient_quantity_log

    def __get_next_ingredient_position(self) -> int:
        print(f"Calibration: {self.calibration_data.positions}")
        for key in self.calibration_data.positions:
            if self.calibration_data.positions[key] is None:
                return key
        return -1

    def run(self) -> None:
        print("\n" + "-" * 80)
        print(f" {self.state} ".center(80))
        print("-" * 80)

        if self.state == RatatouilleStates.HOME:
            self.log(f"Moving to home")
            # if not self.__robot_go_to_joint_state(self.known_poses["joint"]["home"]):
            if not self.__go_to_pose_cartesian_order(
                make_pose(
                    self.known_poses["cartesian"]["home"][:3],
                    self.known_poses["cartesian"]["home"][3:],
                ),
                acceleration_scaling_factor=0.1,
                reverse=True,
            ):
                self.error_message = "Unable to move to joint state"
                self.state = RatatouilleStates.LOG_ERROR
                return

            if self.has_container:
                self.state = RatatouilleStates.REPLACE_CONTAINER
            elif self.__get_next_ingredient_position() == -1:
                # calibration complete, all shelf positions have been populated
                self.state = RatatouilleStates.STOP
            else:
                self.ingredient_id = self.__get_next_ingredient_position()
                print(f"Self ingredient id {self.ingredient_id}")
                self.ingredient_position = list(
                    filter(
                        lambda x: x["id"] == self.ingredient_id,
                        self.known_poses["cartesian"]["ingredients"],
                    )
                )[0]
                # go to next position
                self.state = RatatouilleStates.VISIT_NEXT_CONTAINER

        elif self.state == RatatouilleStates.WRITE_CALIBRATION_DATA:
            print(f"Writing calibration data...")

            print(self.calibration_data.positions)

            # skip writing null values
            _temp_data = {k: v for k,v in self.calibration_data.positions.items() if v}

            with open(
                os.path.join(self.config_dir_path, _INVENTORY_FILE_PATH), "w"
            ) as _temp:
                documents = yaml.dump(_temp_data, _temp)
                print(documents)

            self.state = RatatouilleStates.HOME

        elif self.state == RatatouilleStates.VISIT_NEXT_CONTAINER:
            # Move to ingredient view position
            self.log(f"Moving to ingredient view position for [{self.ingredient_id}]")
            print(
                f"self ingredient expected pose{self.ingredient_position['view_pose']}"
            )
            if not self.__go_to_pose_cartesian_order(
                offset_pose(
                    make_pose(
                        self.ingredient_position["view_pose"][:3],
                        self.ingredient_position["view_pose"][3:],
                    ),
                    _OFFSET_CONTAINER_VIEW,
                ),
                acceleration_scaling_factor=0.1,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            # Compute pregrasp position

            # Compute a pose at origin in pre-grasp frame w.r.t base_link
            self.log("Computing pregrasp pose in world frame")
            marker_origin = Pose()
            marker_origin.position.x = 0.00
            marker_origin.position.y = 0.00
            marker_origin.position.z = 0.00
            quat = quaternion_from_euler(0, 0, 0)
            marker_origin.orientation.x = quat[0]
            marker_origin.orientation.y = quat[1]
            marker_origin.orientation.z = quat[2]
            marker_origin.orientation.w = quat[3]

            start_time = time.time()
            is_marker_detected: bool = False
            self.container_pregrasp_pose = None
            while (
                not is_marker_detected
                and time.time() < start_time + _MARKER_DETECTION_TIMEOUT_SECONDS
            ):
                time.sleep(0.1)
                self.container_pregrasp_pose = (
                    self.pose_transformer.transform_pose_to_frame(
                        pose_source=marker_origin,
                        header_frame_id="pregrasp_" + str(self.ingredient_id),
                        base_frame_id="base_link",
                    )
                )
                if self.container_pregrasp_pose is not None:
                    is_marker_detected = True
                    break

            if not is_marker_detected:
                self.error_message = f"Unable to find ingredient marker."
                self.state = RatatouilleStates.LOG_ERROR
                return

            self.state = RatatouilleStates.LABEL_INGREDIENT

        elif self.state == RatatouilleStates.LABEL_INGREDIENT:
            # Debugging code to bypass verfication
            # self.state = RatatouilleStates.PICK_CONTAINER
            # return
            # End debugging code to bypass verfication

            start_time = time.time()

            rospy.wait_for_service("ingredient_validation")
            try:
                service_call = rospy.ServiceProxy(
                    "ingredient_validation", ValidateIngredient
                )
                response = service_call()
                self.ingredient_name = response.found_ingredient.lower()
            except rospy.ServiceException as e:
                self.error_message = (
                    f"Ingredient detection service call failed. Error: {e}"
                )
                self.state = RatatouilleStates.LOG_ERROR

            self.log(f"Found [{self.ingredient_name}]")
            self.state = RatatouilleStates.PICK_CONTAINER

        elif self.state == RatatouilleStates.PICK_CONTAINER:
            self.log("Opening gripper")
            self.__robot_open_gripper(wait=False)

            # Move to pregrasp position
            self.log("Moving to pregrasp position")
            if not self.__robot_go_to_pose_goal(
                pose=self.container_pregrasp_pose.pose, acc_scaling=0.1
            ):
                self.error_message = "Error moving to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            # Compute actual container position from pre-grasp position
            # Compute pose of container using fixed offset from the pre-grasp frame
            # w.r.t base_link
            pose_marker_wrist_frame = Pose()
            pose_marker_wrist_frame.position.z = 0.175
            pose_marker_wrist_frame.orientation.w = 1

            pose_marker_base_frame = self.pose_transformer.transform_pose_to_frame(
                pose_source=pose_marker_wrist_frame,
                header_frame_id="wrist_3_link",
                base_frame_id="base_link",
            )

            # correct gripper angling upward issue by adding pitch correction to tilt
            # the gripper upward
            pose_marker_base_frame.pose = self.__correct_gripper_angle_tilt(
                pose_marker_base_frame.pose
            )

            # save actual container position to access later in PICK_CONTAINER state
            self.ingredient_actual_position = pose_marker_base_frame.pose

            # Move to container position
            self.log("Moving to pick container from actual container position")
            if not self.__robot_go_to_pose_goal(
                pose=self.ingredient_actual_position, acc_scaling=0.05
            ):
                self.error_message = "Error moving to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            # Grip the container
            self.log("Closing the gripper")
            self.__robot_close_gripper(wait=True)

            self.has_container = True

            # Move to expected ingredient position
            self.log("Moving to expected ingredient position")
            temp_pose = make_pose(
                [
                    self.ingredient_position["view_pose"][0],
                    self.ingredient_position["view_pose"][1],
                    self.ingredient_position["view_pose"][2] + _CONTAINER_LIFT_OFFSET,
                ],
                self.ingredient_position["view_pose"][3:],
            )
            temp_pose = self.__correct_gripper_angle_tilt(temp_pose)
            if not self.__robot_go_to_pose_goal(pose=temp_pose, acc_scaling=0.05):
                self.error_message = "Error moving to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            # go back out of shelf
            self.log("Backing out of the shelf")
            if not self.__robot_go_to_pose_goal(
                offset_pose(
                    self.robot_mg.get_current_pose(),
                    [0, _CONTAINER_SHELF_BACKOUT_OFFSET, 0],
                ),
                acc_scaling=0.1,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            self.state = RatatouilleStates.CHECK_QUANTITY

        elif self.state == RatatouilleStates.CHECK_QUANTITY:

            self.log("Moving to pre-sense position")
            if not self.__robot_go_to_joint_state(
                self.known_poses["joint"]["pre_sense"]
            ):
                self.error_message = "Unable to move to joint state"
                self.state = RatatouilleStates.LOG_ERROR
                return

            self.log("Moving to sense position")
            if not self.__robot_go_to_pose_goal(
                make_pose(
                    self.known_poses["cartesian"]["sense"][:3],
                    self.known_poses["cartesian"]["sense"][3:],
                ),
                orient_tolerance=0.05,
            ):
                self.error_message = "Unable to move to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            self.__robot_open_gripper(wait=True)

            self.ingredient_quantity = 100

            self.__robot_close_gripper(wait=True)

            self.log("Moving to pre-sense position")
            if not self.__robot_go_to_joint_state(
                self.known_poses["joint"]["pre_sense"]
            ):
                self.error_message = "Unable to move to joint state"
                self.state = RatatouilleStates.LOG_ERROR
                return

            # self.log("Wait for weight estimate from force-torque sensor")
            # time.sleep(2)
            # self.ingredient_quantity: Float64 = rospy.wait_for_message(
            #     "force_torque_weight", Float64, timeout=None
            # )
            # self.ingredient_quantity = self.ingredient_quantity.data * 1000

            self.log(f"Estimated weight: {self.ingredient_quantity}")

            self.__robot_go_to_joint_state(self.known_poses["joint"]["home"])

            self.state = RatatouilleStates.HOME

        elif self.state == RatatouilleStates.REPLACE_CONTAINER:

            # write data
            self.calibration_data.positions[self.ingredient_id] = Container(
                self.ingredient_name,
                self.ingredient_quantity,
            )

            # correct the z-height of the container_expected_pose using container_observed_pose z-height
            self.ingredient_position[2] = self.ingredient_actual_position.position.z

            # Move up a little to prevent container hitting the shelf
            self.log(
                "Moving a little above expected view (to avoid hitting shelf while replacing container)"
            )
            if not self.__go_to_pose_cartesian_order(
                offset_pose(
                    make_pose(
                        self.ingredient_position["view_pose"][:3],
                        self.ingredient_position["view_pose"][3:],
                    ),
                    [0, _CONTAINER_SHELF_BACKOUT_OFFSET, _CONTAINER_LIFT_OFFSET],
                ),
                acceleration_scaling_factor=0.1,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            # Move to ingredient position
            self.log("Moving to replace container at ingredient position")

            # revert correction for gripper angle tilt
            _temp_pose = self.__correct_gripper_angle_tilt(
                self.robot_mg.get_current_pose(), reverse=False
            )

            if not self.__robot_go_to_pose_goal(
                offset_pose(
                    _temp_pose,
                    [0, -_CONTAINER_SHELF_BACKOUT_OFFSET, -_CONTAINER_LIFT_OFFSET],
                )
            ):
                self.error_message = "Error moving to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            # Release gripper
            self.log("Opening gripper")
            self.__robot_open_gripper(wait=True)

            # Remove container once replaced
            self.has_container = False

            # Move back out of the shelf
            self.log("Backing out of the shelf")
            if not self.__robot_go_to_pose_goal(
                offset_pose(
                    self.robot_mg.get_current_pose(),
                    [0, _CONTAINER_SHELF_BACKOUT_OFFSET, _CONTAINER_LIFT_OFFSET],
                ),
                acc_scaling=0.1,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            self.state = RatatouilleStates.WRITE_CALIBRATION_DATA

        elif self.state == RatatouilleStates.LOG_ERROR:
            rospy.logerr(self.error_message)
            rospy.logwarn("Press any key to reset.")
            input()
            # reset user request after error
            self.error_message = None
            self.request = None
            self.state = RatatouilleStates.HOME

        return

    def __robot_open_gripper(self, wait):
        if not self.disable_gripper:
            return self.robot_mg.open_gripper(wait=wait)

    def __robot_close_gripper(self, wait):
        if not self.disable_gripper:
            return self.robot_mg.close_gripper(wait=wait)

    def __robot_go_to_joint_state(self, pose):
        if not self.debug_mode:
            return self.robot_mg.go_to_joint_state(pose)

    def __robot_go_to_pose_goal(
        self, pose, acc_scaling=0.2, velocity_scaling=0.2, orient_tolerance=0.01
    ):
        if not self.debug_mode:
            return self.robot_mg.go_to_pose_goal(
                pose,
                acc_scaling=acc_scaling,
                velocity_scaling=velocity_scaling,
                orient_tolerance=orient_tolerance,
            )

    def __correct_gripper_angle_tilt(self, pose: Pose, reverse: bool = False) -> Pose:
        _temp_euler = euler_from_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        angle_offset = 0.04 * (-1, 1)[reverse]
        _temp_quaternion = quaternion_from_euler(
            _temp_euler[0] + angle_offset, _temp_euler[1], _temp_euler[2]
        )
        pose.orientation.x = _temp_quaternion[0]
        pose.orientation.y = _temp_quaternion[1]
        pose.orientation.z = _temp_quaternion[2]
        pose.orientation.w = _temp_quaternion[3]
        return pose

    def __go_to_pose_cartesian_order(
        self,
        goal: Pose,
        acceleration_scaling_factor: float,
        reverse: bool = False,
    ) -> None:

        # go to required orientation
        current_pose = self.robot_mg.get_current_pose()
        if not self.__robot_go_to_pose_goal(
            make_pose(
                [
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z,
                ],
                [
                    goal.orientation.x,
                    goal.orientation.y,
                    goal.orientation.z,
                    goal.orientation.w,
                ],
            ),
            orient_tolerance=0.1,
        ):
            return False

        relative_pose: Pose = offset_pose_relative(
            goal, self.robot_mg.get_current_pose()
        )
        offsets = [
            [relative_pose.position.x, 0, 0],
            [0, 0, relative_pose.position.z],
            [0, relative_pose.position.y, 0],
        ]

        if reverse:
            offsets.reverse()
        for offset in offsets:
            if not self.__robot_go_to_pose_goal(
                offset_pose(self.robot_mg.get_current_pose(), offset),
                acc_scaling=acceleration_scaling_factor,
            ):
                return False
        return True

    def log(self, msg):
        if self.verbose:
            print(msg)
        if self.stop_and_proceed:
            print("(Press enter to continue): ", end="")
            input()

    def reset_position(self):
        self.__robot_go_to_joint_state(self.known_poses["joint"]["home"])


if __name__ == "__main__":
    # start ROS node
    rospy.init_node(_ROS_NODE_NAME)
    ros_rate = rospy.Rate(_ROS_RATE)

    # parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--debug", help="Enable debug mode (run without robot)", action="store_true"
    )
    parser.add_argument("--config-dir", help="Directory path for configuration files")
    parser.add_argument("--dispense-log-file", help="Dispensing log file path")
    parser.add_argument(
        "--ingredient-quantity-log", help="Ingredient quantity log file path"
    )
    parser.add_argument(
        "--disable-gripper", help="Disable gripper commands", action="store_true"
    )
    parser.add_argument(
        "--disable-external-input", help="Disable user input board", action="store_true"
    )
    parser.add_argument("--verbose", help="Enable verbose output", action="store_true")
    parser.add_argument(
        "--stop-and-proceed",
        help="Announce and wait for key-press before performing each action",
        action="store_true",
    )
    args = parser.parse_args()
    # args = parser.parse_args(sys.argv[4:])

    # Set additional options for running in debug mode
    if args.debug:
        args.disable_gripper = True
        args.disable_external_input = True
        args.verbose = True

    if args.stop_and_proceed:
        args.verbose = True

    rospack = rospkg.RosPack()
    package_path = rospack.get_path(_ROS_NODE_NAME)

    if args.config_dir is None:
        args.config_dir = os.path.join(package_path, "config")

    if args.dispense_log_file is None:
        args.dispense_log_file = os.path.join(
            package_path, "logs", "dispense_history.log"
        )
        temp_dir = os.path.dirname(args.dispense_log_file)
        if not os.path.exists(temp_dir):
            os.makedirs(temp_dir)

    if args.ingredient_quantity_log is None:
        args.ingredient_quantity_log = os.path.join(
            package_path, "logs", "ingredient_quantities.log"
        )
        temp_dir = os.path.dirname(args.ingredient_quantity_log)
        if not os.path.exists(temp_dir):
            os.makedirs(temp_dir)

    # initialize state machine
    ratatouille = Ratatouille(
        RatatouilleStates.HOME,
        disable_gripper=args.disable_gripper,
        config_dir_path=args.config_dir,
        verbose=args.verbose,
        stop_and_proceed=args.stop_and_proceed,
        debug_mode=args.debug,
        disable_external_input=args.disable_external_input,
        dispense_log_file=args.dispense_log_file,
        ingredient_quantity_log=args.ingredient_quantity_log,
    )

    # reset robot position on start
    print("Reset position to HOME.")
    ratatouille.reset_position()

    # run state machine while ROS is running
    while not rospy.is_shutdown() and ratatouille.state != RatatouilleStates.STOP:
        ratatouille.run()
        ros_rate.sleep()
