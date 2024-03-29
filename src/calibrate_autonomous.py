#!/usr/bin/env python3

import argparse
import os
import time

import rospkg
import rospy
import yaml
from geometry_msgs.msg import Pose
from ingredient_validation.srv import ValidateIngredient
from motion.commander import RobotMoveGroup
from motion.utils import make_pose, offset_pose, offset_pose_relative
from ratatouille_pose_transforms.transforms import PoseTransforms
from sensor_interface.msg import Weight
from std_msgs.msg import Float64, String
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from tf.transformations import *
from visualization_msgs.msg import Marker

from planner.planner import (
    IngredientTypes,
    InventoryItem,
    InventoryUpdateStates,
    RatatouillePlanner,
)

_ROS_RATE = 10.0
_ROS_NODE_NAME = "ratatouille_planner"
_DISPENSE_THRESHOLD = 50
_INGREDIENT_DETECTION_TIMEOUT_SECONDS = 5
_MARKER_DETECTION_TIMEOUT_SECONDS = 2
_OFFSET_CONTAINER_VIEW = [0.00, 0.19, -0.07]
_CONTAINER_LIFT_OFFSET = 0.015
_CONTAINER_SHELF_BACKOUT_OFFSET = 0.20
_SENSING_STATION_WSCALE_TOPIC = "/sensing_station/weighing_scale"

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
    known_poses = None  # read from file, pose where we view container
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
        bypass_id_service: bool = False,
    ) -> None:

        super().__init__(config_dir_path, debug_mode, disable_gripper, verbose)

        # initialize flags
        self.stop_and_proceed = stop_and_proceed
        self.disable_external_input = disable_external_input
        self.debug_bypass_picking = bypass_picking
        self.debug_bypass_sensing = bypass_sensing
        self.bypass_id_service = bypass_id_service

        # initialize dependencies

        self.pose_transformer = PoseTransforms()
        self.weight_subscriber = rospy.Subscriber(
            _SENSING_STATION_WSCALE_TOPIC, Weight, callback=self.__weight_callback
        )
        self.log("Waiting to receive sensing station weighing scale data.")
        rospy.wait_for_message(_SENSING_STATION_WSCALE_TOPIC, Weight)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        # initialize state variables
        self.state = state
        self.has_container = False
        self.ingredient_id = None
        self.ingredient_position = None
        self.ingredient_name = None
        self.ingredient_quantity = None
        self.error_message = None
        self.weighing_scale_weight = None

        # temporary variables
        self.ingredient_actual_position = None
        self.ingredient_placed_position = None

    def __get_next_ingredient_position(self) -> int:
        for key in self.inventory:
            if self.inventory[key] is None:
                return key
        return -1

    def __weight_callback(self, data: float) -> None:
        self.weighing_scale_weight = data.weight

    def run(self) -> None:
        self.print_current_state_banner()

        if self.state == InventoryUpdateStates.HOME:
            _next_state = None
            if self.has_container:
                _next_state = InventoryUpdateStates.REPLACE_CONTAINER
            elif self.__get_next_ingredient_position() == -1:
                # calibration complete, all shelf positions have been populated
                _next_state = InventoryUpdateStates.STOP
            else:
                self.ingredient_id = self.__get_next_ingredient_position()
                self.ingredient_position = self.known_poses["cartesian"]["positions"][
                    self.ingredient_id
                ]
                # go to next position
                _next_state = InventoryUpdateStates.VISIT_NEXT_CONTAINER

            # skip going to home if next state is visiting next container
            if _next_state != InventoryUpdateStates.VISIT_NEXT_CONTAINER:
                self.log(f"Moving to home")
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
                    self.error_state = self.state
                    self.state = InventoryUpdateStates.LOG_ERROR
                    return

            self.state = _next_state

        elif self.state == InventoryUpdateStates.WRITE_INVENTORY_DATA:

            # update inventory of current position
            self.inventory[self.ingredient_id] = InventoryItem(
                position=self.ingredient_id,
                name=self.ingredient_name,
                quantity=self.ingredient_quantity,
                pose=self.ingredient_placed_position,
            )
            self.write_inventory()
            if self.ingredient_placed_position is not None:
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time.now()

                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                marker.type = 1
                marker.id = self.ingredient_id

                # Set the scale of the marker
                marker.scale.x = 0.10
                marker.scale.y = 0.15
                marker.scale.z = 0.15

                # Set the color
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                
                # Set the pose of the marker
                marker.pose.position.x = self.ingredient_placed_position[0]
                marker.pose.position.y = self.ingredient_placed_position[1] - marker.scale.y
                marker.pose.position.z = self.ingredient_placed_position[2]
                marker.pose.orientation.x = self.ingredient_placed_position[3]
                marker.pose.orientation.y = self.ingredient_placed_position[4]
                marker.pose.orientation.z = self.ingredient_placed_position[5]
                marker.pose.orientation.w = self.ingredient_placed_position[6]
                self.marker_pub.publish(marker) 

                text_marker = Marker()
                text_marker.header.frame_id = "base_link"
                text_marker.header.stamp = rospy.Time.now()

                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                text_marker.type = 9
                text_marker.id = 100 + self.ingredient_id

                # Set the scale of the text_marker
                text_marker.scale.z = 0.03

                # Set the color
                text_marker.color.r = 0.0
                text_marker.color.g = 0.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0
                text_marker.text = self.ingredient_name + "\n" + str(int(self.ingredient_quantity)) + "g"
                # Set the pose of the text_marker
                text_marker.pose.position.x = self.ingredient_placed_position[0]
                text_marker.pose.position.y = self.ingredient_placed_position[1] - text_marker.scale.y
                text_marker.pose.position.z = self.ingredient_placed_position[2] + text_marker.scale.z
                text_marker.pose.orientation.x = self.ingredient_placed_position[3]
                text_marker.pose.orientation.y = self.ingredient_placed_position[4]
                text_marker.pose.orientation.z = self.ingredient_placed_position[5]
                text_marker.pose.orientation.w = self.ingredient_placed_position[6]
                self.marker_pub.publish(text_marker) 

            # reset state variables
            self.ingredient_id = None
            self.ingredient_name = None
            self.ingredient_actual_position = None
            self.ingredient_placed_position = None

            self.state = InventoryUpdateStates.HOME

            # # go directly to next container instead of going home
            # self.ingredient_id = self.__get_next_ingredient_position()
            # self.ingredient_position = self.known_poses["cartesian"]["positions"][
            #     self.ingredient_id
            # ]
            # # go to next position
            # self.state = InventoryUpdateStates.VISIT_NEXT_CONTAINER

        elif self.state == InventoryUpdateStates.VISIT_NEXT_CONTAINER:
            # Move to ingredient view position
            self.log(f"Moving to ingredient view position for [{self.ingredient_id}]")
            print(f"self ingredient expected pose{self.ingredient_position}")
            if not self._go_to_pose_cartesian_order(
                offset_pose(
                    make_pose(
                        self.ingredient_position[:3],
                        self.ingredient_position[3:],
                    ),
                    _OFFSET_CONTAINER_VIEW,
                ),
                acceleration_scaling_factor=0.1,
                velocity_scaling=0.9,
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
                time.sleep(0.01)
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
                self.ingredient_name = IngredientTypes.NO_CONTAINER.__str__()
                self.ingredient_quantity = 0
                self.state = InventoryUpdateStates.WRITE_INVENTORY_DATA

        elif self.state == InventoryUpdateStates.LABEL_INGREDIENT:
            # Debugging code to bypass verfication
            if self.bypass_id_service:
                self.ingredient_name = IngredientTypes.SALT.__str__()
                self.state = InventoryUpdateStates.PICK_CONTAINER
                return

            start_time = time.time()

            rospy.wait_for_service("ingredient_validation")
            try:
                service_call = rospy.ServiceProxy(
                    "ingredient_validation", ValidateIngredient
                )
                response = service_call(mode="rgb")
                print(f"Service Response: {response.found_ingredient}")

            except rospy.ServiceException as e:
                self.error_message = (
                    f"Ingredient detection service call (RGB) failed. Error: {e}"
                )
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR

            try:
                # self.ingredient_name = IngredientTypes(
                #     response.found_ingredient.lower()
                # )
                self.ingredient_name = response.found_ingredient
                print(f"Found: {self.ingredient_name}")
                self.state = InventoryUpdateStates.PICK_CONTAINER

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
                self.state = InventoryUpdateStates.WRITE_INVENTORY_DATA
                return

            self.log("Opening gripper")
            self._robot_open_gripper(wait=False)

            # Move to pregrasp position
            self.log("Moving to pregrasp position")
            if not self._robot_go_to_pose_goal(
                pose=self.container_pregrasp_pose.pose,
                acc_scaling=0.3,
                velocity_scaling=0.9,
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
                pose=self.ingredient_actual_position,
                acc_scaling=0.3,
                velocity_scaling=0.9,
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

            if not self._robot_go_to_pose_goal(
                pose=temp_pose,
                acc_scaling=0.05,
                velocity_scaling=0.5,
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
                acceleration_scaling_factor=0.2,
                velocity_scaling=0.9,
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
            tared_weight = self.weighing_scale_weight
            self.log("Moving to pre-sense position")
            if not self._robot_go_to_joint_state(
                self.known_poses["joint"]["pre_sense"],
                acc_scaling=0.3,
                velocity_scaling=0.9,
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
                acc_scaling=0.1,
                velocity_scaling=0.9,
            ):
                self.error_message = "Unable to move to pose goal"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            self._robot_open_gripper(wait=True)
            # wait for weighing scale readings to settle
            time.sleep(3)
            # 98g container weight
            self.ingredient_quantity = self.weighing_scale_weight - 98 - tared_weight

            # close gripper before spectral service call to ensure good contact against sensor
            self._robot_close_gripper(wait=True)

            # self.ingredient_quantity = 100
            if not self.bypass_id_service:
                rospy.wait_for_service("ingredient_validation")
                try:
                    service_call = rospy.ServiceProxy(
                        "ingredient_validation", ValidateIngredient
                    )
                    response = service_call(
                        mode="spectral",
                        ingredient_name=self.ingredient_name
                        # TODO: remove hack for spectral camera run
                        # mode="spectral",
                        # ingredient_name="salt",
                    )
                    # TODO: assign and log correct response from spectral validation
                    # self.log(f"Spectral camera response: {response.found_ingredient.lower()}")
                    self.ingredient_name = response.found_ingredient
                    print(f"Spectral camera response: {response}")

                except rospy.ServiceException as e:
                    self.error_message = f"Ingredient detection service call (spectral) failed. Error: {e}"
                    self.error_state = self.state
                    self.state = InventoryUpdateStates.LOG_ERROR

            # self._robot_close_gripper(wait=True)

            self.log("Moving to pre-sense position")
            if not self._robot_go_to_joint_state(
                self.known_poses["joint"]["pre_sense"],
                acc_scaling=0.3,
                velocity_scaling=0.9,
            ):
                self.error_message = "Unable to move to joint state"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            self.log(f"Estimated weight: {self.ingredient_quantity}")

            self._robot_go_to_joint_state(
                self.known_poses["joint"]["home"],
                acc_scaling=0.3,
                velocity_scaling=0.9,
            )

            self.state = InventoryUpdateStates.HOME

        elif self.state == InventoryUpdateStates.REPLACE_CONTAINER:

            # correct the z-height of the container_expected_pose using container_observed_pose z-height
            self.ingredient_position[2] = self.ingredient_actual_position.position.z

            # Move up a little to prevent container hitting the shelf
            self.log(
                "Moving a little above expected view (to avoid hitting shelf while replacing container)"
            )
            if not self._go_to_pose_cartesian_order(
                offset_pose(
                    make_pose(
                        self.ingredient_position[:3],
                        self.ingredient_position[3:],
                    ),
                    [0, _CONTAINER_SHELF_BACKOUT_OFFSET, _CONTAINER_LIFT_OFFSET],
                ),
                acceleration_scaling_factor=0.3,
                velocity_scaling=0.9,
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
                ),
                acc_scaling=0.1,
                velocity_scaling=0.9,
            ):
                self.error_message = "Error moving to pose goal"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            # Release gripper
            self.log("Opening gripper")
            self._robot_open_gripper(wait=True)

            _temp: Pose = self.robot_mg.get_current_pose()
            self.ingredient_placed_position = [
                _temp.position.x,
                _temp.position.y,
                _temp.position.z,
                _temp.orientation.x,
                _temp.orientation.y,
                _temp.orientation.z,
                _temp.orientation.w,
            ]

            # Remove container once replaced
            self.has_container = False

            # go back out after placing container to not hit next container
            if not self._go_to_pose_cartesian_order(
                offset_pose(
                    self.robot_mg.get_current_pose(),
                    [0, _CONTAINER_SHELF_BACKOUT_OFFSET, 0],
                ),
                acceleration_scaling_factor=0.3,
                velocity_scaling=0.9,
            ):
                self.error_message = "Error moving to pose goal (backing out of shelf)"
                self.error_state = self.state
                self.state = InventoryUpdateStates.LOG_ERROR
                return

            self.state = InventoryUpdateStates.WRITE_INVENTORY_DATA

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
    parser.add_argument(
        "--bypass-id-service",
        help="Bypass ingredient identification",
        action="store_true",
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
        bypass_id_service=args.bypass_id_service,
    )

    # reset robot position on start
    ratatouille.reset_position()

    # run state machine while ROS is running
    while not rospy.is_shutdown() and ratatouille.state != InventoryUpdateStates.STOP:
        ratatouille.run()
        ros_rate.sleep()
