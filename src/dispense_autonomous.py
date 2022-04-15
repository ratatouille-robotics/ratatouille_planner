#!/usr/bin/env python3

import rospy
import rospkg
import argparse
import yaml
import os
from enum import Enum, auto
from motion.commander import RobotMoveGroup
from ratatouille_pose_transforms.transforms import PoseTransforms


_ROS_RATE = 10.0
_ROS_NODE_NAME = "ratatouille_planner"


class RatatouilleStates(Enum):
    COLD_START = auto()
    WAIT_INPUT = auto()
    MOVING = auto()
    SEARCH_MARKER = auto()
    DISPENSING = auto()
    ERROR = auto()


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
    error_message = None

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
        self.log(f"Executing state: {self.state}")
        self.log("-" * 80)

        if self.state == RatatouilleStates.COLD_START:
            self.log(f"Opening gripper")
            if not self.disable_gripper:
                self.robot_mg.open_gripper()
            self.log(f"Moving to joint pose: {self.known_poses['joint']['home']}")
            self.robot_go_to_joint_state(self.known_poses["joint"]["home"])
            self.state = RatatouilleStates.WAIT_INPUT

        elif self.state == RatatouilleStates.WAIT_INPUT:
            print("Enter input ingredient number. Options are:")
            for index, ingredient in enumerate(
                self.known_poses["cartesian"]["ingredients"]
            ):
                print(index + 1, ingredient)

        elif self.state == RatatouilleStates.MOVING:
            raise NotImplementedError

        elif self.state == RatatouilleStates.SEARCH_MARKER:
            raise NotImplementedError

        elif self.state == RatatouilleStates.DISPENSING:
            raise NotImplementedError

        elif self.state == RatatouilleStates.ERROR:
            print(self.error_message)
            self.state = RatatouilleStates.WAIT_INPUT

        self.log(f"\nNext state: {self.state}")
        self.log("-" * 80)

    def robot_go_to_joint_state(self, args):
        if not self.debug_mode:
            self.robot_mg.go_to_joint_state(args)

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
        RatatouilleStates.COLD_START,
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
