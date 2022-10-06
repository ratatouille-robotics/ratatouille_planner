#!/usr/bin/env python3

import rospy
import rospkg
import argparse
import yaml
import os
from tf.transformations import *
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, String
from typing import List
import time
from datetime import datetime

from motion.utils import make_pose, offset_pose, offset_pose_relative
from dispense.dispense import Dispenser
from sensor_interface.msg import UserInput
from ingredient_validation.srv import ValidateIngredient
from planner.planner import DispensingStates, RatatouillePlanner

_ROS_RATE = 10.0
_ROS_NODE_NAME = "ratatouille_planner"
_DISPENSE_THRESHOLD = 50
_INGREDIENT_DETECTION_TIMEOUT_SECONDS = 5
# _MARKER_DETECTION_TIMEOUT_SECONDS = 5
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


class DispensingStateMachine(RatatouillePlanner):
    # flags
    debug_mode = None
    disable_gripper = None
    verbose = None
    stop_and_proceed = None
    disable_external_input = None
    
    pouring_characteristics = None

    # state variables
    state: DispensingStates = None
    request: DispensingRequest = None
    container: Container = None
    error_message: str = None
    ingredient_quantities: dict = None

    # log files

    def __init__(
        self,
        state: DispensingStates,
        config_dir_path: str,
        disable_gripper: bool = False,
        verbose: bool = False,
        stop_and_proceed: bool = False,
        debug_mode: bool = False,
        disable_external_input: bool = False,
        bypass_dispensing: bool = False
    ) -> None:

        super().__init__(config_dir_path, debug_mode, disable_gripper, verbose)

        # initialize flags
        self.stop_and_proceed = stop_and_proceed
        self.disable_external_input = disable_external_input
        self.debug_bypass_dispensing = bypass_dispensing

        self.load_pouring_characteristics()

        # initialize state variables
        self.state = state
        self.container = None
        self.request = None
        self.error_message = None

    def run(self) -> None:
        self.print_current_state_banner()

        if self.state == DispensingStates.HOME:
            self.log("Moving to home")
            # if not self._robot_go_to_joint_state(self.known_poses["joint"]["home"]):
            if not self._go_to_pose_cartesian_order(
                make_pose(
                    self.known_poses["cartesian"]["home"][:3],
                    self.known_poses["cartesian"]["home"][3:],
                ),
                acceleration_scaling_factor=0.1,
                reverse=True,
            ):
                self.error_message = "Unable to move to joint state"
                self.state = DispensingStates.LOG_ERROR
                return

            if self.container is None:
                if self.request is None:
                    self.state = DispensingStates.AWAIT_USER_INPUT
                else:
                    self.state = DispensingStates.PICK_CONTAINER
            else:
                if self.request is None:
                    self.state = DispensingStates.REPLACE_CONTAINER
                else:
                    self.state = DispensingStates.DISPENSE

        elif self.state == DispensingStates.AWAIT_USER_INPUT:
            if not self.disable_external_input:
                # TODO: send prompt to display menu to external interface
                
                # TODO: receive user input from external interface
                # self.log("Waiting for user input from the input board")
                # user_request: UserInput = rospy.wait_for_message(
                #     "user_input", UserInput, timeout=None
                # )

                # validate input
                # try:
                #     # check if ingredient in list
                #     ingredient = list(
                #         filter(
                #             lambda x: x["name"] == user_request.ingredient.lower(),
                #             self.known_poses["cartesian"]["ingredients"],
                #         )
                #     )[0]
                #     self.request = DispensingRequest(
                #         ingredient_id=ingredient["id"],
                #         ingredient_name=ingredient["name"].lower(),
                #         container_pose=ingredient["view_pose"],
                #         quantity=float(user_request.quantity),
                #     )
                # except:
                #     self.error_message = "Invalid user input."
                raise NotImplementedError
            else:
                # print menu and read input from command line
                print(" INGREDIENTS: ".center(80, "-"))
                for position in self.inventory.positions:
                    if self.inventory.positions[position] is None:
                        continue
                    _ingredient_name = self.inventory.positions[position].name
                    print(f"{position}: {_ingredient_name}")
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
                self.state = DispensingStates.HOME
            else:
                self.state = DispensingStates.LOG_ERROR
            
        elif self.state == DispensingStates.PICK_CONTAINER:
            self._robot_open_gripper(wait=False)

            self.container_pregrasp_pose = offset_pose(
                make_pose(
                    self.ingredient_position["view_pose"][:3],
                    self.ingredient_position["view_pose"][3:],
                ),
                _OFFSET_CONTAINER_VIEW,
            )

            # Move to pregrasp position
            self.log("Moving to pregrasp position")
            if not self._robot_go_to_pose_goal(
                pose=self.container_pregrasp_pose.pose, acc_scaling=0.1
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
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
            pose_marker_base_frame.pose = self._correct_gripper_angle_tilt(
                pose_marker_base_frame.pose
            )

            # save actual container position to access later in PICK_CONTAINER state
            self.container_observed_pose = pose_marker_base_frame.pose

            # Move to container position
            self.log("Moving to pick container from actual container position")
            if not self._robot_go_to_pose_goal(
                pose=self.container_observed_pose, acc_scaling=0.05
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            # Grip the container
            self._robot_close_gripper(wait=True)

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
            temp_pose = self._correct_gripper_angle_tilt(temp_pose)
            if not self._robot_go_to_pose_goal(pose=temp_pose, acc_scaling=0.05):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            # go back out of shelf
            self.log("Backing out of the shelf")
            if not self._robot_go_to_pose_goal(
                offset_pose(
                    self.robot_mg.get_current_pose(),
                    [0, _CONTAINER_SHELF_BACKOUT_OFFSET, 0],
                ),
                acc_scaling=0.1,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            self.state = DispensingStates.HOME

        elif self.state == DispensingStates.DISPENSE:
            # # # TODO-nevalsar Remove
            # self.request = None
            # self.state = DispensingStates.HOME
            # return

            # Move to pre-dispense position
            self.log("Moving to pre-dispense position")
            if not self._robot_go_to_joint_state(
                self.known_poses["joint"]["pre_dispense"]
            ):
                self.error_message = "Unable to move to joint state"
                self.state = DispensingStates.LOG_ERROR
                return

            # Move to dispense position
            self.log(
                f"Moving to dispensing position for ingredient [{self.request.ingredient_name}]"
            )
            dispensing_params = self.pouring_characteristics[
                self.request.ingredient_name
            ]
            _temp = self.known_poses["cartesian"]["pouring"][
                dispensing_params["container"]
            ][dispensing_params["pouring_position"]]
            if not self._robot_go_to_pose_goal(
                make_pose(_temp[:3], _temp[3:]),
                orient_tolerance=0.05,
                velocity_scaling=0.15,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            # Dispense ingredient
            self.log(
                f"Dispensing [{self.request.quantity}] grams of [{self.request.ingredient_name}]"
            )

            # debugging code to bypass dispensing
            if self.debug_bypass_dispensing:
                self.request = None
                self.state = DispensingStates.HOME
                ratatouille._robot_go_to_joint_state(
                    ratatouille.known_poses["joint"]["home"]
                )
                return
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
            self.ingredient_quantities[
                self.request.ingredient_name
            ] -= actual_dispensed_quantity
            self.log(f"Updated ingredient quantities : {self.ingredient_quantities}")

            # Move to pre-dispense position
            self.log("Moving to pre-dispense position")
            if not self._robot_go_to_joint_state(
                self.known_poses["joint"]["pre_dispense"]
            ):
                self.error_message = "Unable to move to joint state"
                self.state = DispensingStates.LOG_ERROR
                return

            # TODO: update inventory

            # Since dispensing is complete, clear user request
            self.request = None
            self.state = DispensingStates.HOME
            ratatouille._robot_go_to_joint_state(
                ratatouille.known_poses["joint"]["home"]
            )

        elif self.state == DispensingStates.REPLACE_CONTAINER:
            # correct the z-height of the container_expected_pose using container_observed_pose z-height
            self.container.container_expected_pose[
                2
            ] = self.container.container_observed_pose.position.z

            # Move up a little to prevent container hitting the shelf
            self.log(
                "Moving a little above expected view (to avoid hitting shelf while replacing container)"
            )
            if not self._go_to_pose_cartesian_order(
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
                self.state = DispensingStates.LOG_ERROR
                return

            # Move to ingredient position
            self.log("Moving to replace container at ingredient position")

            # revert correction for gripper angle tilt
            _temp_pose = self._correct_gripper_angle_tilt(
                self.robot_mg.get_current_pose(), reverse=False
            )

            if not self._robot_go_to_pose_goal(
                offset_pose(
                    _temp_pose,
                    [0, -_CONTAINER_SHELF_BACKOUT_OFFSET, -_CONTAINER_LIFT_OFFSET],
                )
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            # Release gripper
            self.log("Opening gripper")
            self._robot_open_gripper(wait=True)

            # Remove container once replaced
            self.container = None

            # Move back out of the shelf
            self.log("Backing out of the shelf")
            if not self._robot_go_to_pose_goal(
                offset_pose(
                    self.robot_mg.get_current_pose(),
                    [0, _CONTAINER_SHELF_BACKOUT_OFFSET, _CONTAINER_LIFT_OFFSET],
                ),
                acc_scaling=0.1,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            self.state = DispensingStates.HOME

        elif self.state == DispensingStates.LOG_ERROR:
            rospy.logerr(self.error_message)
            rospy.logwarn("Press any key to reset.")
            input()
            # reset user request after error
            self.error_message = None
            self.request = None
            self.state = DispensingStates.HOME

        return

    def load_pouring_characteristics(self):
        for position in self.inventory.positions:
            if self.inventory.positions[position] is None:
                continue
            _ingredient_name = self.inventory.positions[position].name
            with open(
                file=os.path.join(
                    self.config_dir_path,
                    "ingredient_params",
                    _ingredient_name + ".yaml",
                ),
                mode="r",
            ) as f:
                ingredient_params = yaml.safe_load(f)
                self.pouring_characteristics[_ingredient_name] = ingredient_params

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
    parser.add_argument(
        "--bypass-dispensing", help="Bypass dispensing", action="store_true"
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

    if args.stop_and_proceed:
        args.verbose = True

    rospack = rospkg.RosPack()
    package_path = rospack.get_path(_ROS_NODE_NAME)

    if args.config_dir is None:
        args.config_dir = os.path.join(package_path, "config")

    # initialize state machine
    ratatouille = DispensingStateMachine(
        DispensingStates.HOME,
        disable_gripper=args.disable_gripper,
        config_dir_path=args.config_dir,
        verbose=args.verbose,
        stop_and_proceed=args.stop_and_proceed,
        debug_mode=args.debug,
        disable_external_input=args.disable_external_input,
        bypass_dispensing=args.bypass_dispensing
    )

    # reset robot position on start
    ratatouille.reset_position()

    # run state machine while ROS is running
    while not rospy.is_shutdown():
        ratatouille.run()
        ros_rate.sleep()
