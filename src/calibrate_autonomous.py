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
import time

from motion.commander import RobotMoveGroup
from ratatouille_pose_transforms.transforms import PoseTransforms
from motion.utils import make_pose, offset_pose, offset_pose_relative
from ingredient_validation.srv import ValidateIngredient
from planner.planner import (
    InventoryUpdateStates,
    IngredientTypes,
    RatatouillePlanner,
    Container,
)

_ROS_RATE = 10.0
_ROS_NODE_NAME = "ratatouille_planner"
_DISPENSE_THRESHOLD = 50
_INGREDIENT_DETECTION_TIMEOUT_SECONDS = 5
_MARKER_DETECTION_TIMEOUT_SECONDS = 5
_OFFSET_CONTAINER_VIEW = [0.00, 0.20, -0.07]
_CONTAINER_LIFT_OFFSET = 0.015
_CONTAINER_SHELF_BACKOUT_OFFSET = 0.20

# ASSUMPTIONS
# - all markers should be at correct positions (eg. marker 1 at position 1)
# - all containers are of same height, limited by UR5e arm reach (software fully supports it)
# - every container should be filled to minimum fill level for ingredient identification


class InventoryUpdateStateMachine(RatatouillePlanner):
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

    # state variables
    state: InventoryUpdateStates = None
    error_message: str = None

    def __init__(
        self,
        state: InventoryUpdateStates,
        config_dir_path: str,
        disable_gripper: bool = False,
        verbose: bool = False,
        stop_and_proceed: bool = False,
        debug_mode: bool = False,
        disable_external_input: bool = False,
        bypass_picking: bool = False,
        bypass_sensing: bool = False,
    ) -> None:

        super().__init__(config_dir_path, debug_mode, disable_gripper, verbose)

        # initialize flags
        self.stop_and_proceed = stop_and_proceed
        self.disable_external_input = disable_external_input
        self.debug_bypass_picking = bypass_picking
        self.debug_bypass_sensing = bypass_sensing

        # initialize dependencies

        self.pose_transformer = PoseTransforms()

        # initialize state variables
        self.state = state
        self.has_container = False
        self.ingredient_id = None
        self.ingredient_position = None
        self.ingredient_name = None
        self.error_message = None

    def __get_next_ingredient_position(self) -> int:
        print(f"Calibration: {self.inventory.positions}")
        for key in self.inventory.positions:
            if self.inventory.positions[key] is None:
                return key
        return -1

    def run(self) -> None:
        self.print_current_state_banner()

        if self.state == InventoryUpdateStates.HOME:
            self.log(f"Moving to home")
            if not self._go_to_pose_cartesian_order(
                make_pose(
                    self.known_poses["cartesian"]["home"][:3],
                    self.known_poses["cartesian"]["home"][3:],
                ),
                acceleration_scaling_factor=0.1,
                reverse=True,
            ):
                self.error_message = "Unable to move to joint state"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            if self.has_container:
                self.state = InventoryUpdateStates.REPLACE_CONTAINER
            elif self.__get_next_ingredient_position() == -1:
                # calibration complete, all shelf positions have been populated
                self.state = InventoryUpdateStates.STOP
            else:
                self.ingredient_id = self.__get_next_ingredient_position()
                print(f"Self ingredient id {self.ingredient_id}")
                self.ingredient_position = self.known_poses["cartesian"]["positions"][
                    self.ingredient_id
                ]
                # go to next position
                self.state = InventoryUpdateStates.VISIT_NEXT_CONTAINER

        elif self.state == InventoryUpdateStates.WRITE_CALIBRATION_DATA:

            # update inventory of current position
            self.inventory.positions[self.ingredient_id] = Container(
                self.ingredient_name,
                self.ingredient_quantity,
            )
            self.write_inventory()
            self.state = InventoryUpdateStates.HOME

        elif self.state == InventoryUpdateStates.VISIT_NEXT_CONTAINER:
            # Move to ingredient view position
            self.log(f"Moving to ingredient view position for [{self.ingredient_id}]")
            print(
                f"self ingredient expected pose{self.ingredient_position}"
            )
            if not self._go_to_pose_cartesian_order(
                offset_pose(
                    make_pose(
                        self.ingredient_position[:3],
                        self.ingredient_position[3:],
                    ),
                    _OFFSET_CONTAINER_VIEW,
                ),
                acceleration_scaling_factor=0.1,
            ):
                self.error_message = "Error moving to pose goal"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
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

            if is_marker_detected:
                self.state = InventoryUpdateStates.LABEL_INGREDIENT
            else:
                # self.error_message = f"Unable to find ingredient marker."
                self.error_state = self.state
                # self.state = InventoryUpdateStates.LOG_ERROR
                # return
                self.ingredient_name = IngredientTypes.NO_CONTAINER
                self.ingredient_quantity = 0
                self.state = InventoryUpdateStates.WRITE_CALIBRATION_DATA

        elif self.state == InventoryUpdateStates.LABEL_INGREDIENT:
            # Debugging code to bypass verfication
            # self.state = InventoryUpdateStates.PICK_CONTAINER
            # return
            # End debugging code to bypass verfication

            start_time = time.time()

            rospy.wait_for_service("ingredient_validation")
            try:
                service_call = rospy.ServiceProxy(
                    "ingredient_validation", ValidateIngredient
                )
                response = service_call()

            except rospy.ServiceException as e:
                self.error_message = (
                    f"Ingredient detection service call failed. Error: {e}"
                )
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR

            print(f"Service Response: {response.found_ingredient.lower()}")

            try:
                self.ingredient_name = IngredientTypes(
                    response.found_ingredient.lower()
                )
                self.log(f"Found [{self.ingredient_name}]")
                self.state = InventoryUpdateStates.PICK_CONTAINER

                if self.ingredient_name == IngredientTypes.NO_INGREDIENT:
                    self.ingredient_quantity = 0
                    self.state = InventoryUpdateStates.HOME

            except ValueError:
                self.error_message = (
                    f"Cannot parse detected ingredient {response.found_ingredient}"
                )
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR

        elif self.state == InventoryUpdateStates.PICK_CONTAINER:
            if self.debug_bypass_picking:
                self.log(f"Container picking bypassed.")
                self.ingredient_quantity = 100
                self.state = InventoryUpdateStates.WRITE_CALIBRATION_DATA
                return

            self.log("Opening gripper")
            self._robot_open_gripper(wait=False)

            # Move to pregrasp position
            self.log("Moving to pregrasp position")
            if not self._robot_go_to_pose_goal(
                pose=self.container_pregrasp_pose.pose, acc_scaling=0.1
            ):
                self.error_message = "Error moving to pose goal"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
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
            self.ingredient_actual_position = pose_marker_base_frame.pose

            # Move to container position
            self.log("Moving to pick container from actual container position")
            if not self._robot_go_to_pose_goal(
                pose=self.ingredient_actual_position, acc_scaling=0.05
            ):
                self.error_message = "Error moving to pose goal"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            # Grip the container
            self.log("Closing the gripper")
            self._robot_close_gripper(wait=True)

            self.has_container = True

            # Move to expected ingredient position
            self.log("Moving to expected ingredient position")
            temp_pose = make_pose(
                [
                    self.ingredient_position[0],
                    self.ingredient_position[1],
                    self.ingredient_position[2] + _CONTAINER_LIFT_OFFSET,
                ],
                self.ingredient_position[3:],
            )

            # TODO: verify if second correction required
            # temp_pose = self._correct_gripper_angle_tilt(temp_pose)

            if not self._robot_go_to_pose_goal(pose=temp_pose, acc_scaling=0.05):
                self.error_message = "Error moving to pose goal"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
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
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            # go home in cartesian order
            if not self._go_to_pose_cartesian_order(
                make_pose(
                    self.known_poses["cartesian"]["home"][:3],
                    self.known_poses["cartesian"]["home"][3:],
                ),
                acceleration_scaling_factor=0.1,
                reverse=True,
            ):
                self.error_message = "Unable to move to joint state"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            self.state = InventoryUpdateStates.CHECK_QUANTITY

        elif self.state == InventoryUpdateStates.CHECK_QUANTITY:

            # bypass sensing
            if self.debug_bypass_sensing:
                self.log(f"Sensing bypassed.")
                self.ingredient_quantity = 100
                self.state = InventoryUpdateStates.HOME
                return
            # end bypass sensing

            self.log("Moving to pre-sense position")
            if not self._robot_go_to_joint_state(
                self.known_poses["joint"]["pre_sense"]
            ):
                self.error_message = "Unable to move to joint state"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            self.log("Moving to sense position")
            if not self._robot_go_to_pose_goal(
                make_pose(
                    self.known_poses["cartesian"]["sense"][:3],
                    self.known_poses["cartesian"]["sense"][3:],
                ),
                orient_tolerance=0.05,
            ):
                self.error_message = "Unable to move to pose goal"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            # wait before opening gripper
            # time.sleep(5);

            self._robot_open_gripper(wait=True)

            self.ingredient_quantity = 100

            self._robot_close_gripper(wait=True)

            self.log("Moving to pre-sense position")
            if not self._robot_go_to_joint_state(
                self.known_poses["joint"]["pre_sense"]
            ):
                self.error_message = "Unable to move to joint state"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            # self.log("Wait for weight estimate from force-torque sensor")
            # time.sleep(2)
            # self.ingredient_quantity: Float64 = rospy.wait_for_message(
            #     "force_torque_weight", Float64, timeout=None
            # )
            # self.ingredient_quantity = self.ingredient_quantity.data * 1000

            self.log(f"Estimated weight: {self.ingredient_quantity}")

            self._robot_go_to_joint_state(self.known_poses["joint"]["home"])

            self.state = InventoryUpdateStates.HOME

        elif self.state == InventoryUpdateStates.REPLACE_CONTAINER:

            # correct the z-height of the container_expected_pose using container_observed_pose z-height
            self.ingredient_position["view_pose"][
                2
            ] = self.ingredient_actual_position.position.z

            # Move up a little to prevent container hitting the shelf
            self.log(
                "Moving a little above expected view (to avoid hitting shelf while replacing container)"
            )
            if not self._go_to_pose_cartesian_order(
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
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
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
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            # Release gripper
            self.log("Opening gripper")
            self._robot_open_gripper(wait=True)

            # Remove container once replaced
            self.has_container = False

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
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            self.state = InventoryUpdateStates.WRITE_CALIBRATION_DATA

        elif self.state == InventoryUpdateStates.LOG_ERROR:
            if self.error_state == InventoryUpdateStates.CHECK_QUANTITY:
                exit(0)
            rospy.logerr(self.error_message)
            rospy.logwarn("Press any key to reset.")
            input()
            # reset user request after error
            self.error_state = None
            self.error_message = None
            self.request = None
            self.state = InventoryUpdateStates.HOME

        return


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
    parser.add_argument(
        "--bypass-picking", help="Bypass container picking", action="store_true"
    )
    parser.add_argument("--bypass-sensing", help="Bypass sensing", action="store_true")
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

    # initialize state machine
    ratatouille = InventoryUpdateStateMachine(
        InventoryUpdateStates.HOME,
        disable_gripper=args.disable_gripper,
        config_dir_path=args.config_dir,
        verbose=args.verbose,
        stop_and_proceed=args.stop_and_proceed,
        debug_mode=args.debug,
        disable_external_input=args.disable_external_input,
        bypass_picking=args.bypass_picking,
        bypass_sensing=args.bypass_sensing,
    )

    # reset robot position on start
    ratatouille.reset_position()

    # run state machine while ROS is running
    while not rospy.is_shutdown() and ratatouille.state != InventoryUpdateStates.STOP:
        ratatouille.run()
        ros_rate.sleep()
