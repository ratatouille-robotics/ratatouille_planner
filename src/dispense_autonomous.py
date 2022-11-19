#!/usr/bin/env python3

import argparse
import os
import sys
import time
from collections import deque
from datetime import datetime
from typing import List
from urllib import request

import actionlib
import rospkg
import rospy
import yaml
from dispense.dispense import Dispenser
from geometry_msgs.msg import Pose
from ingredient_validation.srv import ValidateIngredient
from motion.utils import make_pose, offset_pose, offset_pose_relative
from sensor_interface.msg import UserInput
from std_msgs.msg import Float64, String
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from tf.transformations import *

from planner.planner import (
    DispensingDelay,
    DispensingRequest,
    DispensingStates,
    IngredientTypes,
    RatatouillePlanner,
    RecipeAction,
)
from ratatouille_planner.msg import (
    RecipeRequestAction,
    RecipeRequestFeedback,
    RecipeRequestResult,
)

_ROS_RATE = 10.0
_ROS_NODE_NAME = "ratatouille_planner"
_OFFSET_CONTAINER_VIEW = [0.00, 0.20, -0.07]
_CONTAINER_LIFT_OFFSET = 0.015
_CONTAINER_SHELF_BACKOUT_OFFSET = 0.20
_CONTAINER_PREGRASP_OFFSET_Z = 0.175
_MAX_USER_INPUT_WAIT_BEFORE_ERROR_SECONDS = 300


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
    request: List[RecipeAction] = None
    container: int = None
    error_message: str = None
    ingredient_quantities: dict = None

    def __init__(
        self,
        state: DispensingStates,
        config_dir_path: str,
        disable_gripper: bool = False,
        verbose: bool = False,
        stop_and_proceed: bool = False,
        debug_mode: bool = False,
        disable_external_input: bool = False,
        bypass_dispensing: bool = False,
    ) -> None:

        super().__init__(config_dir_path, debug_mode, disable_gripper, verbose)

        # Register and start action server to recieve recipe requests
        self.action_server = actionlib.SimpleActionServer(
            "RecipeRequest",
            RecipeRequestAction,
            execute_cb=self._action_server_queue_request,
            auto_start=False,
        )
        self.action_server.start()

        # initialize flags
        self.stop_and_proceed = stop_and_proceed
        self.disable_external_input = disable_external_input
        self.debug_bypass_dispensing = bypass_dispensing

        self.load_pouring_characteristics()

        # initialize state variables
        self.state = state
        self.status_container = -1
        self.status_request = -1
        self.request = deque()
        self.error_message = None

    def _action_server_queue_request(self, goal):
        # read from goal and add to self.request
        print(f"Received request for recipe ID: [{goal}]")
        try:
            _selected_recipe = self._get_recipe_by_id(goal.recipe_id)
            assert _selected_recipe is not None
            self._add_request_from_recipe(_selected_recipe)
            self.print_recipe(_selected_recipe)
            # self.action_server.set_succeeded()
            self.action_server.publish_feedback(
                RecipeRequestFeedback(percent_complete=15)
            )
            percent_complete = 0
            while (
                not rospy.is_shutdown()
                and not self.action_server.is_preempt_requested()
                and percent_complete < 100
            ):
                percent_complete += 1
                self.action_server.publish_feedback(
                    RecipeRequestFeedback(percent_complete=percent_complete)
                )
                time.sleep(0.1)
            self.action_server.set_succeeded(RecipeRequestResult("Done"))
        except Exception:
            print("Error: Missing ingredients/insufficient quantity in inventory.")
            self.action_server.set_aborted()

    def _get_recipe_by_id(self, recipe_id: int) -> dict:
        for recipe in self.recipes:
            if recipe["id"] == recipe_id:
                return recipe
        return None

    def _add_request_from_recipe(self, recipe: dict) -> List[DispensingRequest]:
        assert recipe is not None and recipe["ingredients"] is not None

        # add dispensing requests for each ingredient in recipe
        for _ingredient in recipe["ingredients"]:
            # if delay step, add delay action to recipe
            if _ingredient["name"] == "_wait":
                self.request.append(DispensingDelay(_ingredient["quantity"]))
                continue

            # check if ingredient is in inventory
            # check if sufficient quantity is in inventory
            _temp_id = self.get_position_of_ingredient(_ingredient["name"])
            if (
                _temp_id is None
                or self.inventory.positions[_temp_id].quantity < _ingredient["quantity"]
            ):
                raise  # "Missing ingredients/insufficient quantity in inventory."
            request = DispensingRequest(
                ingredient_id=_temp_id,
                ingredient_name=self.inventory.positions[_temp_id].name,
                quantity=_ingredient["quantity"],
                ingredient_pose=self.inventory.positions[_temp_id].pose,
            )
            self.request.append(request)

    def print_recipe(self, recipe):
        self.log(f"Selected recipe: {recipe['name']}".center(80))
        self.log(f"{'-' * 40}".center(80))
        for _ingredient in recipe["ingredients"]:
            _temp_id = self.get_position_of_ingredient(_ingredient["name"])
            self.log(
                f"Ingredient ({_temp_id}) {_ingredient['name']}: {_ingredient['quantity']} grams".center(
                    80
                )
            )
        self.log(f"{'-' * 40}".center(80))

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
                acceleration_scaling_factor=0.3,
                velocity_scaling=0.9,
                reverse=True,
            ):
                self.error_message = "Unable to move to joint state"
                self.state = DispensingStates.LOG_ERROR
                return

            if self.status_container < 0:
                if len(self.request) == 0:
                    self.state = DispensingStates.AWAIT_USER_INPUT
                else:
                    if type(self.request[0]) == DispensingDelay:
                        self.state = DispensingStates.WAIT
                    else:
                        self.state = DispensingStates.PICK_CONTAINER
            else:
                if len(self.request) == 0:
                    self.state = DispensingStates.REPLACE_CONTAINER
                elif self.status_request != self.request[0].guid:
                    self.status_request = self.request[0].guid
                    self.state = DispensingStates.REPLACE_CONTAINER
                else:
                    self.state = DispensingStates.DISPENSE

        elif self.state == DispensingStates.WAIT:
            time.sleep(self.request[0].duration)
            self.request.popleft()
            self.status_request = self.request[0].guid
            self.state = DispensingStates.HOME

        elif self.state == DispensingStates.AWAIT_USER_INPUT:
            if not self.disable_external_input:
                # waiting for user input from web interface

                wait_for_user_input_start = time.time()
                # self.request is updated directly in self.action_server_callback
                while not rospy.is_shutdown() and len(self.request) == 0:
                    if (
                        time.time() - wait_for_user_input_start
                        > _MAX_USER_INPUT_WAIT_BEFORE_ERROR_SECONDS
                    ):
                        self.error_message = "No user input received"
                        self.state = DispensingStates.LOG_ERROR
                        return

            else:
                # print menu and read input from command line

                print(" MENU".center(80, "-"))
                for recipe in self.recipes:
                    print(f"Recipe ID {recipe['id']}: {recipe['name']}".center(80))
                print("-" * 80)

                # read and parse user input
                try:
                    _selected_recipe_id = int(input("Enter recipe ID: "))
                    _selected_recipe = self._get_recipe_by_id(_selected_recipe_id)
                    assert _selected_recipe is not None
                except:
                    self.error_message = "Unable to parse user input."

                try:
                    self._add_request_from_recipe(_selected_recipe)
                    assert len(self.request) > 0
                    self.status_request = self.request[
                        0
                    ].guid  # index of first ingredient to be dispenses
                    self.print_recipe(_selected_recipe)
                except:
                    self.error_message = "Missing ingredient/ insufficient quantity - cannot prepare recipe."

            if self.error_message is None:
                # print selected recipe

                self.log(
                    f"Picking [{self.request[0].quantity} grams] of [{self.request[0].ingredient_name}]"
                )
                self.state = DispensingStates.HOME
            else:
                self.state = DispensingStates.LOG_ERROR

        elif self.state == DispensingStates.PICK_CONTAINER:
            self._robot_open_gripper(wait=False)

            # Move to container position
            self.log("Moving to pick container from container position")
            _temp = self.inventory.positions[self.request[0].ingredient_id].pose
            if not self._go_to_pose_cartesian_order(
                goal=make_pose(
                    _temp[:3],
                    _temp[3:],
                ),
                acceleration_scaling_factor=0.3,
                velocity_scaling=0.9,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            # Grip the container
            self._robot_close_gripper(wait=True)

            # Set container
            self.status_container = self.request[0].ingredient_id

            # Lift to avoid hitting shelf while going to HOME orientation
            self.log("Lifting container above shelf position")
            if not self._robot_go_to_pose_goal(
                offset_pose(
                    self.robot_mg.get_current_pose(),
                    [0, 0, _CONTAINER_LIFT_OFFSET],
                ),
                acc_scaling=0.1,
                velocity_scaling=0.9,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            self.state = DispensingStates.HOME

        elif self.state == DispensingStates.DISPENSE:

            # Move to pre-dispense position
            self.log("Moving to pre-dispense position")
            if not self._robot_go_to_joint_state(
                self.known_poses["joint"]["pre_dispense"],
                acc_scaling=0.3,
                velocity_scaling=0.9,
            ):
                self.error_message = "Unable to move to joint state"
                self.state = DispensingStates.LOG_ERROR
                return

            # debugging code to bypass dispensing
            if self.debug_bypass_dispensing:
                self.request.popleft()
                self.state = DispensingStates.HOME
                ratatouille._robot_go_to_joint_state(
                    ratatouille.known_poses["joint"]["home"],
                    acc_scaling=0.3,
                    velocity_scaling=0.9,
                )
                return
            # End debugging code to bypass dispensing

            # Move to dispense position
            self.log(
                f"Moving to dispensing position for ingredient [{self.request[0].ingredient_name}]"
            )
            dispensing_params = self.pouring_characteristics[
                self.request[0].ingredient_name
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
                f"Dispensing [{self.request[0].quantity}] grams of [{self.request[0].ingredient_name}]"
            )

            dispenser = Dispenser(self.robot_mg)
            actual_dispensed_quantity = dispenser.dispense_ingredient(
                dispensing_params, float(self.request[0].quantity), log_data=True
            )
            actual_dispensed_quantity = float(actual_dispensed_quantity)

            dispense_error = actual_dispensed_quantity - self.request[0].quantity
            self.log(
                f"Dispensed [{actual_dispensed_quantity}] grams with error of [{dispense_error}] (requested [{self.request.quantity}] grams)"
            )

            # update ingredient quantity in inventory
            self.inventory.positions[
                self.request[0].ingredient_id
            ].quantity -= actual_dispensed_quantity
            self.write_inventory()
            self.log(
                f"Updated ingredient quantities : {self.inventory.positions[self.request[0].ingredient_id]}"
            )

            # Move to pre-dispense position
            self.log("Moving to pre-dispense position")
            if not self._robot_go_to_joint_state(
                self.known_poses["joint"]["pre_dispense"],
                acc_scaling=0.3,
                velocity_scaling=0.9,
            ):
                self.error_message = "Unable to move to joint state"
                self.state = DispensingStates.LOG_ERROR
                return

            # Since dispensing is complete, clear user request
            self.request.popleft()
            self.state = DispensingStates.HOME
            ratatouille._robot_go_to_joint_state(
                ratatouille.known_poses["joint"]["home"],
                acc_scaling=0.3,
                velocity_scaling=0.9,
            )

        elif self.state == DispensingStates.REPLACE_CONTAINER:
            # Move up a little to prevent container hitting the shelf
            self.log(
                "Moving a little above expected view to avoid hitting shelf while replacing container)"
            )

            _temp = self.inventory.positions[self.status_container].pose
            # if recovering from error, use previous container id
            if not self._go_to_pose_cartesian_order(
                offset_pose(
                    make_pose(_temp[:3], _temp[3:]),
                    [0, _CONTAINER_SHELF_BACKOUT_OFFSET, _CONTAINER_LIFT_OFFSET],
                ),
                acceleration_scaling_factor=0.3,
                velocity_scaling=0.9,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            # Move to ingredient position
            self.log("Moving to replace container at ingredient position")

            # revert correction for gripper angle tilt
            # _temp_pose = self._correct_gripper_angle_tilt(
            #     self.robot_mg.get_current_pose(), reverse=False
            # )
            _temp_pose = self.robot_mg.get_current_pose()

            if not self._robot_go_to_pose_goal(
                offset_pose(
                    _temp_pose,
                    [0, -_CONTAINER_SHELF_BACKOUT_OFFSET, -_CONTAINER_LIFT_OFFSET],
                ),
                acc_scaling=0.3,
                velocity_scaling=0.9,
            ):
                self.error_message = "Error moving to pose goal"
                self.state = DispensingStates.LOG_ERROR
                return

            # _temp = self.inventory.positions[self.status_container].pose
            # if not self._go_to_pose_cartesian_order(
            #     make_pose(_temp[:3], _temp[3:]),
            #     acceleration_scaling_factor=0.3,
            #     velocity_scaling=0.9,
            # ):
            #     self.error_message = "Error moving to pose goal"
            #     self.state = DispensingStates.LOG_ERROR
            #     return

            # Release gripper
            self.log("Opening gripper")
            self._robot_open_gripper(wait=True)

            # Remove container once replaced
            self.status_container = -1

            self.state = DispensingStates.HOME

        elif self.state == DispensingStates.LOG_ERROR:
            rospy.logerr(self.error_message)
            rospy.logwarn("Press any key to reset.")
            input()
            # reset user request after error
            self.error_message = None
            self.request.clear()
            self.state = DispensingStates.HOME

        return

    def load_pouring_characteristics(self):
        self.pouring_characteristics = {}
        for ingredient in IngredientTypes:
            with open(
                file=os.path.join(
                    self.config_dir_path, "ingredient_params", f"{ingredient}.yaml"
                ),
                mode="r",
            ) as f:
                ingredient_params = yaml.safe_load(f)
                self.pouring_characteristics[ingredient] = ingredient_params


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
        bypass_dispensing=args.bypass_dispensing,
    )

    # reset robot position on start
    ratatouille.reset_position()

    # run state machine while ROS is running
    while not rospy.is_shutdown():
        ratatouille.run()
        ros_rate.sleep()
