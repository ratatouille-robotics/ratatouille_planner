#!/usr/bin/env python3

import rospy
import rospkg
import argparse
import yaml
import os
import time
from enum import Enum, auto
from motion.commander import RobotMoveGroup
from ratatouille_pose_transforms.transforms import PoseTransforms


_ROS_RATE = 10.0
_ROS_NODE_NAME = "ratatouille_planner"


class RatatouilleStates(Enum):
    HOME = auto()
    AWAIT_USER_INPUT = auto()
    MOVING = auto()
    SEARCH_MARKER = auto()
    DISPENSING = auto()


class Ratatouille:
    # flags
    debug_mode = None
    disable_gripper = None
    verbose = None
    stop_and_proceed = None

    # dependencies
    robot_mg = None
    known_poses = None
    pose_transformer = None

    # state variables
    state = None
    target_ingredient_id = None
    target_quantity = None  # in grams

    def __init__(
        self,
        state: RatatouilleStates,
        pose_file_path: str,
        disable_gripper: bool = False,
        verbose=False,
        stop_and_proceed=False,
        debug_mode=False,
    ) -> None:
        # initialize flags
        self.debug_mode = debug_mode
        self.disable_gripper = disable_gripper
        self.verbose = verbose
        self.stop_and_proceed = stop_and_proceed

        # initialize dependencies
        if not self.debug_mode:
            self.robot_mg = RobotMoveGroup()
        with open(
            file=pose_file_path,
            mode="r",
        ) as _temp:
            self.known_poses = yaml.safe_load(_temp)
        self.pose_transformer = PoseTransforms()

        # initialize state variables
        self.state = state

    def run(self) -> None:
        self.log("\n" + "-" * 80)
        self.log(f" {self.state} ".center(80))
        self.log("-" * 80)

        if self.state == RatatouilleStates.HOME:
            self.log(f"Opening gripper")
            self.__robot_open_gripper(wait=True)
            self.log(f"Moving to HOME pose")
            self.__robot_go_to_joint_state(self.known_poses["joint"]["home"])

            self.state = RatatouilleStates.AWAIT_USER_INPUT

        elif self.state == RatatouilleStates.AWAIT_USER_INPUT:
            print(" INGREDIENTS: ".center(80, "-"))
            for index, ingredient in enumerate(
                self.known_poses["cartesian"]["ingredients"]
            ):
                print(f"{index + 1}: {ingredient}")
            print("-" * 80)
            _raw_input_ingredient_id = input("Enter ingredient number: ")
            _raw_input_ingredient_quantity = input("Enter quantity in grams: ")

            # validate user input
            try:
                self.target_ingredient_id = -1 + int(
                    _raw_input_ingredient_id
                )  # to account for zero indexing
                self.target_quantity = int(_raw_input_ingredient_quantity)
            except:
                self.logerr(
                    "Unable to parse user input. Please enter valid ingredient ID and quantity."
                )
                return

            self.state = RatatouilleStates.SEARCH_MARKER

        elif self.state == RatatouilleStates.MOVING:
            raise NotImplementedError

        elif self.state == RatatouilleStates.SEARCH_MARKER:
            raise NotImplementedError

        elif self.state == RatatouilleStates.DISPENSING:
            raise NotImplementedError

        return

    def __robot_open_gripper(self, args):
        if not self.disable_gripper:
            self.robot_mg.open_gripper(args)

    def __robot_go_to_joint_state(self, args):
        if not self.debug_mode:
            self.robot_mg.go_to_joint_state(args)

    def log(self, msg):
        if self.verbose:
            print(msg)
        if self.stop_and_proceed:
            input()

    def logerr(self, error_message):
        rospy.logerr(error_message)
        rospy.logwarn("Press any key to reset.")
        input()
        self.state = RatatouilleStates.HOME


if __name__ == "__main__":
    # start ROS node
    rospy.init_node(_ROS_NODE_NAME)
    ros_rate = rospy.Rate(_ROS_RATE)

    # parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--debug", help="Enable debug mode (run without robot)", action="store_true"
    )
    parser.add_argument("--pose-file", help="File path of pose configuration")
    parser.add_argument(
        "--disable-gripper", help="Disable gripper commands", action="store_true"
    )
    parser.add_argument("--verbose", help="Enable verbose output", action="store_true")
    parser.add_argument(
        "--stop-and-proceed",
        help="Announce and wait for key-press before performing each action",
        action="store_true",
    )
    args = parser.parse_args()

    # disable gripper motion and verbose output if running in debug mode
    if args.debug:
        args.disable_gripper = True
        args.verbose = True

    if args.pose_file is None:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(_ROS_NODE_NAME)
        args.pose_file = os.path.join(package_path, "config", "poses.yml")

    # initialize state machine
    ratatouille = Ratatouille(
        RatatouilleStates.HOME,
        disable_gripper=args.disable_gripper,
        pose_file_path=args.pose_file,
        verbose=args.verbose,
        stop_and_proceed=args.stop_and_proceed,
        debug_mode=args.debug,
    )

    # start state machine while ROS is active
    while not rospy.is_shutdown():
        ratatouille.run()
        ros_rate.sleep()
