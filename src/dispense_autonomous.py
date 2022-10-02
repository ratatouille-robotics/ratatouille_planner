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
from typing import List
import time
from datetime import datetime

from motion.commander import RobotMoveGroup
from ratatouille_pose_transforms.transforms import PoseTransforms
from motion.utils import make_pose, offset_pose, offset_pose_relative
from dispense.dispense import Dispenser
from sensor_interface.msg import UserInput
from ingredient_validation.srv import ValidateIngredient

_ROS_RATE = 10.0
_ROS_NODE_NAME = "ratatouille_planner"
_DISPENSE_THRESHOLD = 50
_INGREDIENT_DETECTION_TIMEOUT_SECONDS = 5
_MARKER_DETECTION_TIMEOUT_SECONDS = 5
_OFFSET_CONTAINER_VIEW = [0.00, 0.20, -0.07]
_CONTAINER_LIFT_OFFSET = 0.015
_CONTAINER_SHELF_BACKOUT_OFFSET = 0.20


class Container:
    ingredient_id: int = None
    ingredient_name: str = None
    container_expected_pose: List[float] = None
    container_pregrasp_pose: PoseStamped = None
    container_observed_pose: List[float] = None

    def __init__(
        self,
        ingredient_id: int,
        ingredient_name: str,
        container_expected_pose: List[float],
        container_pregrasp_pose: PoseStamped,
        container_observed_pose: List[float],
    ) -> None:
        self.ingredient_name = ingredient_name
        self.id = ingredient_id
        self.container_expected_pose = container_expected_pose
        self.container_pregrasp_pose = container_pregrasp_pose
        self.container_observed_pose = container_observed_pose


class DispensingRequest:
    ingredient_id: int = None
    ingredient_name: str = None
    container_expected_pose: List[float] = None
    container_pregrasp_pose: Pose = None
    quantity: float = None

    def __init__(
        self,
        ingredient_id: int,
        ingredient_name: str,
        quantity: float,
        container_pose: List[float],
    ) -> None:
        self.ingredient_id = ingredient_id
        self.ingredient_name = ingredient_name
        self.quantity = quantity
        self.container_expected_pose = container_pose
        self.container_pregrasp_pose = None


class RatatouilleStates(Enum):
    # RESET = auto()
    HOME = auto()
    AWAIT_USER_INPUT = auto()
    SEARCH_MARKER = auto()
    VERIFY_INGREDIENT = auto()
    PICK_CONTAINER = auto()
    CHECK_QUANTITY = auto()
    DISPENSE = auto()
    REPLACE_CONTAINER = auto()
    LOG_ERROR = auto()


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
    request: DispensingRequest = None
    container: Container = None
    error_message: str = None
    ingredient_quantities: dict = None

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
        # initialize flags
        self.debug_mode = debug_mode
        self.disable_gripper = disable_gripper
        self.verbose = verbose
        self.stop_and_proceed = stop_and_proceed
        self.disable_external_input = disable_external_input

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
        self.container = None
        self.request = None
        self.error_message = None

        # initialize log file paths
        self.dispense_log_file = dispense_log_file
        self.ingredient_quantity_log = ingredient_quantity_log

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

            if self.container is None:
                if self.request is None:
                    self.state = RatatouilleStates.AWAIT_USER_INPUT
                else:
                    self.state = RatatouilleStates.SEARCH_MARKER
            else:
                if self.request is None:
                    self.state = RatatouilleStates.REPLACE_CONTAINER
                else:
                    self.state = RatatouilleStates.DISPENSE

        elif self.state == RatatouilleStates.AWAIT_USER_INPUT:
            if not self.disable_external_input:
                self.log("Sending ingredient list to user input board")
                _send_msg = String(
                    "$".join(
                        [str(x) for x in list(self.ingredient_quantities.values())]
                    )
                )
                self.dispensing_update_publisher.publish(_send_msg)

                self.log("Waiting for user input from the input board")
                user_request: UserInput = rospy.wait_for_message(
                    "user_input", UserInput, timeout=None
                )

                try:
                    # check if ingredient in list
                    ingredient = list(
                        filter(
                            lambda x: x["name"] == user_request.ingredient.lower(),
                            self.known_poses["cartesian"]["ingredients"],
                        )
                    )[0]
                    self.request = DispensingRequest(
                        ingredient_id=ingredient["id"],
                        ingredient_name=ingredient["name"].lower(),
                        container_pose=ingredient["view_pose"],
                        quantity=float(user_request.quantity),
                    )
                except:
                    self.error_message = "Invalid user input."
            else:
                # print menu and read input from command line
                print(" INGREDIENTS: ".center(80, "-"))
                for ingredient in self.known_poses["cartesian"]["ingredients"]:
                    print(f"{ingredient['id']}: {ingredient['name']}")
                print("-" * 80)
                _raw_input_ingredient_id = input("Enter ingredient number: ")
                _raw_input_ingredient_quantity = input("Enter quantity in grams: ")

                # validate user input
                try:
                    ingredient = list(
                        filter(
                            lambda x: x["id"] == int(_raw_input_ingredient_id),
                            self.known_poses["cartesian"]["ingredients"],
                        )
                    )[0]
                    self.request = DispensingRequest(
                        ingredient_id=ingredient["id"],
                        ingredient_name=ingredient["name"].lower(),
                        container_pose=ingredient["view_pose"],
                        quantity=float(_raw_input_ingredient_quantity),
                    )
                except:
                    self.error_message = "Unable to parse user input."

            if self.error_message is None:
                self.log(
                    f"User requested for [{self.request.quantity} grams] of [{self.request.ingredient_name}]"
                )
                self.state = RatatouilleStates.HOME
            else:
                self.state = RatatouilleStates.LOG_ERROR

        elif self.state == RatatouilleStates.SEARCH_MARKER:
            # Move to ingredient view position
            self.log(
                f"Moving to ingredient view position for [{self.request.ingredient_name}]"
            )
            # if not self.__robot_go_to_pose_goal(
            #     pose=make_pose(
            #         self.request.container_expected_pose[:3],
            #         self.request.container_expected_pose[3:],
            #     ),
            #     acc_scaling=0.1,
            # )
            if not self.__go_to_pose_cartesian_order(
                offset_pose(
                    make_pose(
                        self.request.container_expected_pose[:3],
                        self.request.container_expected_pose[3:],
                    ),
                    _OFFSET_CONTAINER_VIEW,
                ),
                acceleration_scaling_factor=0.1,
            ):
                # if not self.__robot_go_to_pose_goal(
                #     offset_pose(self.robot_mg.get_current_pose(), _OFFSET_CONTAINER_VIEW),
                #     acc_scaling=0.1,
                # ):
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
                        header_frame_id="pregrasp_" + str(self.request.ingredient_id),
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

            self.state = RatatouilleStates.VERIFY_INGREDIENT

        elif self.state == RatatouilleStates.VERIFY_INGREDIENT:
            # Debugging code to bypass verfication
            self.state = RatatouilleStates.PICK_CONTAINER
            return
            # End debugging code to bypass verfication

            start_time = time.time()

            # TODO: remove
            if self.request.ingredient_name == "lentils":
                self.log(f"Ingredient [{self.request.ingredient_name}] verified.")
                self.state = RatatouilleStates.PICK_CONTAINER
                return

            rospy.wait_for_service("ingredient_validation")
            try:
                service_call = rospy.ServiceProxy(
                    "ingredient_validation", ValidateIngredient
                )
                response = service_call()
                detected_ingredient = response.found_ingredient.lower()
            except rospy.ServiceException as e:
                self.error_message = (
                    f"Ingredient detection service call failed. Error: {e}"
                )
                self.state = RatatouilleStates.LOG_ERROR

            self.log(
                f"Expected [{self.request.ingredient_name}]. Detected [{detected_ingredient}]"
            )

            if detected_ingredient == self.request.ingredient_name:
                self.log(f"Ingredient [{self.request.ingredient_name}] verified.")
                self.state = RatatouilleStates.PICK_CONTAINER
            else:
                self.error_message = (
                    f"Ingredient [{self.request.ingredient_name}] not detected."
                )
                self.state = RatatouilleStates.LOG_ERROR

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
            self.container_observed_pose = pose_marker_base_frame.pose

            # Move to container position
            self.log("Moving to pick container from actual container position")
            if not self.__robot_go_to_pose_goal(
                pose=self.container_observed_pose, acc_scaling=0.05
            ):
                self.error_message = "Error moving to pose goal"
                self.state = RatatouilleStates.LOG_ERROR
                return

            # Grip the container
            self.log("Closing the gripper")
            self.__robot_close_gripper(wait=True)

            # Set container
            self.container = Container(
                ingredient_name=self.request.ingredient_name,
                ingredient_id=self.request.ingredient_id,
                container_expected_pose=self.request.container_expected_pose,
                container_pregrasp_pose=self.container_pregrasp_pose,
                container_observed_pose=self.container_observed_pose,
            )

            # Move to expected ingredient position
            self.log("Moving to expected ingredient position")
            temp_pose = make_pose(
                [
                    self.request.container_expected_pose[0],
                    self.request.container_expected_pose[1],
                    self.request.container_expected_pose[2] + _CONTAINER_LIFT_OFFSET,
                ],
                self.request.container_expected_pose[3:],
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

            self.state = RatatouilleStates.HOME
            # TODO-nevalsar: Remove
            # self.state = RatatouilleStates.REPLACE_CONTAINER

        # elif self.state == RatatouilleStates.CHECK_QUANTITY:
        #     # TODO-nevalsar: Remove
        #     # self.error_message = f"Insufficient quantity"
        #     # self.state = RatatouilleStates.LOG_ERROR
        #     # return

        #     self.log("Wait for weight estimate from force-torque sensor")
        #     time.sleep(2)
        #     weight_estimate: Float64 = rospy.wait_for_message(
        #         "force_torque_weight", Float64, timeout=None
        #     )
        #     weight_estimate = weight_estimate.data * 1000

        #     self.log(f"Estimated weight: {weight_estimate}")
        #     self.log(f"Requested weight: {self.request.quantity}")

        #     # mark dispensing complete to replace container in shelf
        #     if weight_estimate < self.request.quantity + _DISPENSE_THRESHOLD:
        #         self.error_message = f"Insufficient quantity"
        #         self.state = RatatouilleStates.LOG_ERROR
        #     else:
        #         self.state = RatatouilleStates.HOME

        elif self.state == RatatouilleStates.DISPENSE:
            # # # TODO-nevalsar Remove
            # self.request = None
            # self.state = RatatouilleStates.HOME
            # return

            # Move to pre-dispense position
            self.log("Moving to pre-dispense position")
            if not self.robot_mg.go_to_joint_state(
                self.known_poses["joint"]["pre_dispense"],
                cartesian_path=False,
                velocity_scaling=0.15,
            ):
                self.error_message = "Unable to move to joint state"
                self.state = RatatouilleStates.LOG_ERROR
                return
            if not self.robot_mg.go_to_joint_state(
                self.known_poses["joint"]["pre_dispense"],
                cartesian_path=True,
                velocity_scaling=0.15,
            ):
                self.error_message = "Unable to move to joint state"
                self.state = RatatouilleStates.LOG_ERROR
                return

            # # Move to dispense position
            # self.log(
            #     f"Moving to dispensing position for ingredient [{self.request.ingredient_name}]"
            # )
            dispensing_params = self.pouring_characteristics[
                self.request.ingredient_name
            ]
            # _temp = self.known_poses["cartesian"]["pouring"][
            #     dispensing_params["container"]
            # ][dispensing_params["pouring_position"]]
            # pre_dispense_pose = make_pose(_temp[:3], _temp[3:])
            # rospy.loginfo(pre_dispense_pose)
            # if not self.__robot_go_to_pose_goal(
            #     pre_dispense_pose,
            #     orient_tolerance=0.5,
            #     cartesian_path=True,
            #     velocity_scaling=0.3,
            # ):
            #     self.error_message = "Error moving to pose goal"
            #     self.state = RatatouilleStates.LOG_ERROR
            #     return
                
            # assert self.robot_mg.go_to_pose_goal(
            #     pre_dispense_pose,
            #     cartesian_path=True,
            #     orient_tolerance=0.05,
            #     velocity_scaling=0.3,
            # )
            # Dispense ingredient
            self.log(
                f"Dispensing [{self.request.quantity}] grams of [{self.request.ingredient_name}]"
            )

            # debugging code to bypass dispensing
            # self.request = None
            # self.state = RatatouilleStates.HOME
            # ratatouille.__robot_go_to_joint_state(
            #     ratatouille.known_poses["joint"]["home"]
            # )
            # return
            # End debugging code to bypass dispensing

            dispenser = Dispenser(self.robot_mg)
            actual_dispensed_quantity = dispenser.dispense_ingredient(
                dispensing_params, float(self.request.quantity), log_data=True
            )
            actual_dispensed_quantity = float(actual_dispensed_quantity)

            dispense_error = actual_dispensed_quantity - self.request.quantity
            self.log(
                f"Dispensed [{actual_dispensed_quantity}] grams with error of [{dispense_error}] (requested [{self.request.quantity}] grams)"
            )

            # update ingredient quantities
            # self.ingredient_quantities[
            #     self.request.ingredient_name
            # ] -= actual_dispensed_quantity
            # self.log(f"Updated ingredient quantities : {self.ingredient_quantities}")

            # # Move to pre-dispense position
            # self.log("Moving to pre-dispense position")
            # if not self.__robot_go_to_joint_state(
            #     self.known_poses["joint"]["pre_dispense"]
            # ):
            #     self.error_message = "Unable to move to joint state"
            #     self.state = RatatouilleStates.LOG_ERROR
            #     return

            # add entry to dispensing log file
            # with open(self.dispense_log_file, "a+") as dispense_log_file:
            #     dispense_log_file.write(
            #         f"{datetime.now()} - [{self.request.ingredient_name}] - Expected: [{self.request.quantity}], Actual: [{actual_dispensed_quantity}], Error: [{dispense_error}]\n"
            #     )

            # # add entry to dispensing log file
            # with open(self.ingredient_quantity_log, "w+") as dispense_log_file:
            #     dispense_log_file.write(
            #         f"{datetime.now()}\n{self.ingredient_quantities}\n"
            #     )

            # Since dispensing is complete, clear user request
            self.request = None
            self.state = RatatouilleStates.HOME
            ratatouille.__robot_go_to_joint_state(
                ratatouille.known_poses["joint"]["home"]
            )

        elif self.state == RatatouilleStates.REPLACE_CONTAINER:

            # # Move to ingredient view position
            # self.log(
            #     f"Moving to view position for ingredient: {self.container.ingredient_name}"
            # )
            # if not self.__robot_go_to_pose_goal(
            #     make_pose(
            #         self.container.container_expected_pose[:3],
            #         self.container.container_expected_pose[3:],
            #     ),
            #     acc_scaling=0.1,
            # ):
            #     self.error_message = "Error moving to pose goal"
            #     self.state = RatatouilleStates.LOG_ERROR
            #     return

            # # Move up a little to prevent container hitting the shelf
            # self.log("Moving up to avoid hitting shelf while replacing container")
            # if not self.__robot_go_to_pose_goal(
            #     offset_pose(self.robot_mg.get_current_pose(), [0, 0, 0.12]),
            #     acc_scaling=0.1,
            # ):
            #     self.error_message = "Error moving to pose goal"
            #     self.state = RatatouilleStates.LOG_ERROR
            #     return

            # correct the z-height of the container_expected_pose using container_observed_pose z-height
            self.container.container_expected_pose[
                2
            ] = self.container.container_observed_pose.position.z

            # Move up a little to prevent container hitting the shelf
            self.log(
                "Moving a little above expected view (to avoid hitting shelf while replacing container)"
            )
            if not self.__go_to_pose_cartesian_order(
                offset_pose(
                    make_pose(
                        self.container.container_expected_pose[:3],
                        self.container.container_expected_pose[3:],
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
            self.container = None

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

            self.state = RatatouilleStates.HOME

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
        self,
        pose,
        acc_scaling=0.2,
        velocity_scaling=0.2,
        orient_tolerance=0.01,
        cartesian_path=True,
    ):
        if not self.debug_mode:
            return self.robot_mg.go_to_pose_goal(
                pose,
                acc_scaling=acc_scaling,
                velocity_scaling=velocity_scaling,
                orient_tolerance=orient_tolerance,
                cartesian_path=cartesian_path,
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
        # offsets = [
        #     make_pose([current_pose.position.x, current_pose.position.y, current_pose.position.z], current_pose.pose),
        #     make_pose([current_pose.position.x, current_pose.position.y, current_pose.position.z], current_pose.pose),
        #     make_pose([current_pose.position.x, current_pose.position.y, current_pose.position.z], current_pose.pose),

        # ]
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
    while not rospy.is_shutdown():
        ratatouille.run()
        ros_rate.sleep()
