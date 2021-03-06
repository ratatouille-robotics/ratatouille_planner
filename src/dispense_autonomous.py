#!/usr/bin/env python3

import rospy
import rospkg
import argparse
import yaml
import os
from enum import Enum, auto
from tf.transformations import *
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, String
from typing import List

from motion.commander import RobotMoveGroup
from ratatouille_pose_transforms.transforms import PoseTransforms
from motion.utils import make_pose, offset_pose
from dispense.dispense import Dispenser
from sensor_interface.msg import UserInput


_ROS_RATE = 10.0
_ROS_NODE_NAME = "ratatouille_planner"


class Container:
    ingredient_id: int = None
    ingredient_name: str = None

    def __init__(self, ingredient_id, ingredient_name) -> None:
        self.ingredient_name = ingredient_name
        self.id = ingredient_id


class DispensingRequest:
    ingredient_id: int = None
    ingredient_name: str = None
    container_pose: List[float] = None
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
        self.container_pose = container_pose


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
    sensordata = None

    # state variables
    state: RatatouilleStates = None
    request: DispensingRequest = None
    container: Container = None
    error_message: str = None

    def __init__(
        self,
        state: RatatouilleStates,
        config_dir_path: str,
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
            file=os.path.join(config_dir_path, "poses.yml"),
            mode="r",
        ) as _temp:
            self.known_poses = yaml.safe_load(_temp)
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

        # initialize state variables
        self.state = state
        self.container = None
        self.request = None
        self.error_message = None

    def run(self) -> None:
        self.log("\n" + "-" * 80)
        self.log(f" {self.state} ".center(80))
        self.log("-" * 80)

        if self.state == RatatouilleStates.HOME:
            self.log(f"Moving to home")
            self.__robot_go_to_joint_state(self.known_poses["joint"]["home"])

            if self.container is None:
                if self.request is None:
                    self.state = RatatouilleStates.AWAIT_USER_INPUT
                else:
                    self.state = RatatouilleStates.SEARCH_MARKER
            else:
                if self.request is None:
                    self.state = RatatouilleStates.REPLACE_CONTAINER
                else:
                    self.state = RatatouilleStates.CHECK_QUANTITY

        elif self.state == RatatouilleStates.AWAIT_USER_INPUT:
            if not self.disable_external_input:
                self.log("Waiting for user input from the input board")
                user_request: UserInput = rospy.wait_for_message(
                    "user_input", UserInput, timeout=None
                )

                try:
                    # check if ingredient in list
                    ingredient = ingredient_name = list(
                        filter(
                            lambda x: x["name"] == user_request.ingredient,
                            self.known_poses["cartesian"]["ingredients"],
                        )
                    )[0]
                    self.request = DispensingRequest(
                        ingredient_id=ingredient["id"],
                        ingredient_name=ingredient["name"],
                        container_pose=ingredient["pose"],
                        quantity=float(user_request.quantity),
                    )
                except:
                    self.error_message = "Invalid user input."
            else:
                # print menu and read input from command line
                print(" INGREDIENTS: ".center(80, "-"))
                for index, ingredient in enumerate(
                    self.known_poses["cartesian"]["ingredients"]
                ):
                    print(f"{index + 1}: {ingredient['name']}")
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
                        ingredient_name=ingredient["name"],
                        container_pose=ingredient["pose"],
                        quantity=float(_raw_input_ingredient_quantity),
                    )
                except:
                    self.error_message = "Unable to parse user input."

            if self.error_message is None:
                self.state = RatatouilleStates.HOME
            else:
                self.state = RatatouilleStates.LOG_ERROR

        elif self.state == RatatouilleStates.SEARCH_MARKER:
            # Move to ingredient view position
            self.log(
                f"Moving to ingredient view position: {self.request.ingredient_name}"
            )
            self.__robot_go_to_pose_goal(
                pose=make_pose(
                    self.request.container_pose[:3],
                    self.request.container_pose[3:],
                )
            )

            # Compute pregrasp position

            # Compute a pose at origin in pre-grasp frame w.r.t base_link
            self.log("Computing pregrasp position")
            marker_origin = Pose()
            marker_origin.position.x = 0.00
            marker_origin.position.y = 0.00
            marker_origin.position.z = 0.00
            quat = quaternion_from_euler(0, 0, 0)
            marker_origin.orientation.x = quat[0]
            marker_origin.orientation.y = quat[1]
            marker_origin.orientation.z = quat[2]
            marker_origin.orientation.w = quat[3]

            target_pose = self.pose_transformer.transform_pose_to_frame(
                pose_source=marker_origin,
                header_frame_id="pregrasp_" + str(self.request.ingredient_id),
                base_frame_id="base_link",
            )
            if target_pose is None:
                self.error_message = f"Unable to find ingredient marker."

            if self.error_message is None:
                self.state = RatatouilleStates.VERIFY_INGREDIENT
            else:
                self.state = RatatouilleStates.LOG_ERROR

        elif self.state == RatatouilleStates.VERIFY_INGREDIENT:
            detected_ingredient: String = rospy.wait_for_message(
                "detected_ingredient", String, timeout=None
            )

            # mark dispensing complete to replace container in shelf
            if self.request.ingredient_name != detected_ingredient:
                self.state = RatatouilleStates.LOG_ERROR
            else:
                self.state = RatatouilleStates.PICK_CONTAINER

        elif self.state == RatatouilleStates.PICK_CONTAINER:
            # Move to pregrasp position
            self.log("Moving to pregrasp position")
            self.__robot_go_to_pose_goal(pose=target_pose.pose)

            #  Compute container position

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
            _temp_euler = euler_from_quaternion(
                (
                    pose_marker_base_frame.pose.orientation.x,
                    pose_marker_base_frame.pose.orientation.y,
                    pose_marker_base_frame.pose.orientation.z,
                    pose_marker_base_frame.pose.orientation.w,
                )
            )
            _temp_quaternion = quaternion_from_euler(
                _temp_euler[0] + 0.04, _temp_euler[1], _temp_euler[2]
            )
            pose_marker_base_frame.pose.orientation.x = _temp_quaternion[0]
            pose_marker_base_frame.pose.orientation.y = _temp_quaternion[1]
            pose_marker_base_frame.pose.orientation.z = _temp_quaternion[2]
            pose_marker_base_frame.pose.orientation.w = _temp_quaternion[3]

            # Move to container position
            self.log("Moving to pick container to container position")
            self.__robot_go_to_pose_goal(pose=pose_marker_base_frame.pose)

            # Grip the container
            self.log("Closing the gripper")
            self.__robot_close_gripper(wait=True)

            # Move to actual/true ingredient position
            self.log("Moving to true ingredient position")
            pose = make_pose(
                [
                    self.target_ingredient["pose"][0],
                    self.target_ingredient["pose"][1] - 0.20,
                    self.target_ingredient["pose"][2] + 0.05,
                ],
                self.target_ingredient["pose"][3:],
            )
            self.__robot_go_to_pose_goal(pose=pose)

            # go back out of shelf
            self.log("Backing out of the shelf")
            self.__robot_go_to_pose_goal(
                offset_pose(self.robot_mg.get_current_pose(), [0.00, 0.20, 0.00])
            )

            self.container = Container(
                name=self.request.ingredient_name, id=self.request.ingredient_id
            )
            self.state = RatatouilleStates.HOME

        elif self.state == RatatouilleStates.CHECK_QUANTITY:
            weight_ft: Float64 = rospy.wait_for_message(
                "force_torque_weight", Float64, timeout=None
            )

            # mark dispensing complete to replace container in shelf
            if weight_ft < self.request.quantity:
                self.state = RatatouilleStates.DISPENSE
            else:
                self.state = RatatouilleStates.HOME

        elif self.state == RatatouilleStates.DISPENSE:
            # Move to pre-dispense position
            self.log("Moving to pre-dispense position")
            self.__robot_go_to_joint_state(self.known_poses["joint"]["pre_dispense"])

            # Move to dispense position
            self.log(
                f"Moving to dispensing position for ingredient [{self.target_ingredient['name']}]"
            )
            dispensing_params = self.pouring_characteristics[
                self.target_ingredient["name"]
            ]
            pos, orient = self.known_poses["cartesian"]["pouring"][
                dispensing_params["container"]
            ][dispensing_params["pouring_position"]]
            self.__robot_go_to_pose_goal(make_pose(pos, orient))

            # Dispense ingredient
            self.log("Dispensing ingredient")
            dispenser = Dispenser(self.robot_mg)
            dispenser.dispense_ingredient(
                dispensing_params, float(self.target_quantity)
            )

            # Move to pre-dispense position
            self.log("Moving to pre-dispense position")
            self.__robot_go_to_joint_state(self.known_poses["joint"]["pre_dispense"])

            # Remove completed dispense requests
            self.request = None
            self.state = RatatouilleStates.HOME

        elif self.state == RatatouilleStates.REPLACE_CONTAINER:

            # Move to ingredient view position
            self.log(
                f"Moving to view position for ingredient: {self.target_ingredient['name']}"
            )
            self.__robot_go_to_pose_goal(
                make_pose(
                    self.target_ingredient["pose"][:3],
                    self.target_ingredient["pose"][3:],
                )
            )

            # Move up a little to prevent container hitting the shelf
            self.log("Moving up to avoid hitting shelf while replacing container")
            self.__robot_go_to_pose_goal(
                offset_pose(self.robot_mg.get_current_pose(), [0, 0, 0.075])
            )

            # Move to ingredient position
            self.log("Moving to replace container at ingredient position")
            self.__robot_go_to_pose_goal(
                offset_pose(self.robot_mg.get_current_pose(), [0, -0.2, -0.05])
            )

            # Release gripper
            self.log("Opening gripper")
            self.__robot_open_gripper()

            # Remove container once replaced
            self.container = None

            # Move back out of the shelf
            self.log("Backing out of the shelf")
            self.__robot_go_to_pose_goal(
                offset_pose(self.robot_mg.get_current_pose(), [0, 0.20, 0.05])
            )

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
            self.robot_mg.open_gripper(wait=wait)

    def __robot_close_gripper(self, wait):
        if not self.disable_gripper:
            self.robot_mg.close_gripper(wait=wait)

    def __robot_go_to_joint_state(self, pose):
        if not self.debug_mode:
            self.robot_mg.go_to_joint_state(pose)

    def __robot_go_to_pose_goal(self, pose):
        if not self.debug_mode:
            self.robot_mg.go_to_pose_goal(
                pose,
                cartesian_path=True,
                acc_scaling=0.1,
            )

    def log(self, msg):
        if self.verbose:
            print(msg)
        if self.stop_and_proceed:
            input()


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

    # Set additional options for running in debug mode
    if args.debug:
        args.disable_gripper = True
        args.disable_external_input = True
        args.verbose = True

    if args.config_dir is None:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(_ROS_NODE_NAME)
        args.config_dir = os.path.join(package_path, "config")

    # initialize state machine
    ratatouille = Ratatouille(
        RatatouilleStates.HOME,
        disable_gripper=args.disable_gripper,
        config_dir_path=args.config_dir,
        verbose=args.verbose,
        stop_and_proceed=args.stop_and_proceed,
        debug_mode=args.debug,
        disable_external_input=args.disable_external_input,
    )

    # run state machine while ROS is running
    while not rospy.is_shutdown():
        ratatouille.run()
        ros_rate.sleep()
