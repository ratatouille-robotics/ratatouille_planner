from abc import ABC, abstractmethod
import yaml
import os

from enum import Enum, auto
from typing import Dict, List
from geometry_msgs.msg import Pose
from tf.transformations import *

from motion.commander import RobotMoveGroup
from ratatouille_pose_transforms.transforms import PoseTransforms
from motion.utils import make_pose, offset_pose, offset_pose_relative

_INVENTORY_FILE_PATH = "inventory.yaml"
_RECIPE_FILE_PATH = "recipes.yaml"
_CALIBRATION_START_CONTAINER = 1
_CALIBRATION_END_CONTAINER = 15


class IngredientTypes(str, Enum):
    OLIVES_BLACK = "blackolives"
    BLACK_PEPPER = "blackpepper"
    CABBAGE = "cabbage"
    CARROT = "carrot"
    CHERRY_TOMATOES = "cherrytomatoes"
    CHILLI_FLAKES = "chilliflakes"
    CORN = "corn"
    CUCUMBER = "cucumber"
    OLIVES_GREEN = "greenolives"
    HABANERO = "habaneropepper"
    MUSHROOM = "mushroom"
    OREGANO = "oregano"
    PEANUTS = "peanuts"
    ONION_RED = "redonion"
    SALT = "salt"
    SUGAR = "sugar"
    VINEGAR = "vinegar"
    WATER = "water"
    ONION_WHITE = "whiteonion"
    NO_INGREDIENT = "no_ingredient"
    NO_CONTAINER = "no_container"

    def __str__(self) -> str:
        return str.__str__(self)


class DispensingStates(Enum):
    HOME = auto()
    AWAIT_USER_INPUT = auto()
    WAIT = auto()
    PICK_CONTAINER = auto()
    DISPENSE = auto()
    REPLACE_CONTAINER = auto()
    LOG_ERROR = auto()


class InventoryUpdateStates(Enum):
    HOME = auto()
    WRITE_INVENTORY_DATA = auto()
    VISIT_NEXT_CONTAINER = auto()
    LABEL_INGREDIENT = auto()
    PICK_CONTAINER = auto()
    CHECK_QUANTITY = auto()
    REPLACE_CONTAINER = auto()
    LOG_ERROR = auto()
    STOP = auto()


class Container(yaml.YAMLObject):
    yaml_tag = "!container"
    name: str = None
    quantity: float = None
    pose: List[float] = None

    def __init__(
        self, _name: IngredientTypes, _quantity: float, observed_pose: List[float]
    ):
        self.name = str(_name)
        self.quantity = _quantity
        self.pose = observed_pose

    def __repr__(self) -> str:
        return f"name: {self.name}, quantity: {self.quantity}"


yaml.add_path_resolver("!container", ["Container"], dict)


class Shelf(yaml.YAMLObject):
    yaml_tag = "!shelf"
    positions: Dict[int, Container] = None

    def __init__(self, _start_container: int, _end_container: int):
        self.positions = {
            key: None for key in range(_start_container, _end_container + 1)
        }


yaml.add_path_resolver("!shelf", ["Shelf"], dict)


class RatatouillePlanner(ABC):
    debug_mode = False
    disable_gripper = False
    verbose = True
    config_dir_path = None

    inventory = None

    pose_transformer = None
    robot_mg = None
    known_poses = None
    recipes = None
    inventory = Shelf(_CALIBRATION_START_CONTAINER, _CALIBRATION_END_CONTAINER)

    def __init__(
        self,
        _config_dir_path: str,
        _debug: bool,
        _disable_gripper: bool,
        _verbose: bool,
    ) -> None:
        self.config_dir_path = _config_dir_path
        self.debug_mode = _debug
        self.verbose = _verbose
        self.disable_gripper = _disable_gripper

        self.pose_transformer = PoseTransforms()

        if not self.debug_mode:
            self.robot_mg = RobotMoveGroup()

        self.load_known_positions()
        self.load_inventory()
        self.load_recipes()

    @abstractmethod
    def run(self) -> None:
        pass

    def load_known_positions(self):
        with open(
            file=os.path.join(self.config_dir_path, "poses.yaml"),
            mode="r",
        ) as _temp:
            self.known_poses = yaml.safe_load(_temp)

    def load_recipes(self):
        with open(
            file=os.path.join(self.config_dir_path, _RECIPE_FILE_PATH),
            mode="r",
        ) as _temp:
            self.recipes = yaml.load(_temp)

    def load_inventory(self):
        with open(
            file=os.path.join(self.config_dir_path, _INVENTORY_FILE_PATH),
            mode="r",
        ) as _temp:
            _inventory = yaml.load(_temp)
            print(f"Inventory: {_inventory}")
            if _inventory:
                for key in _inventory:
                    self.inventory.positions[key] = _inventory[key]

    def get_position_of_ingredient(self, ingredient_name: str):
        try:
            _temp = list(
                filter(
                    lambda x: x[1].name == ingredient_name,
                    self.inventory.positions.items(),
                )
            )[0]
        except:
            return None
        return _temp[0]

    def write_inventory(self) -> None:
        # skip writing null values
        _inventory = {k: v for k, v in self.inventory.positions.items() if v}
        with open(
            os.path.join(self.config_dir_path, _INVENTORY_FILE_PATH), "w"
        ) as _temp:
            yaml.dump(_inventory, _temp)
        if self.verbose:
            print(f"Inventory updated on disk: {_inventory}")

    def print_current_state_banner(self):
        print("\n" + "-" * 80)
        print(f" {self.state} ".center(80))
        print("-" * 80)

    def log(self, msg):
        if self.verbose:
            print(msg)
        if self.stop_and_proceed:
            print("(Press enter to continue): ", end="")
            input()

    def reset_position(self):
        self.log("Resetting robot position to Home.")
        if self.known_poses == None:
            raise Exception("Poses not loaded.")
        self._robot_go_to_joint_state(self.known_poses["joint"]["home"])

    def _robot_go_to_joint_state(self, pose,velocity_scaling: float=0.2,acc_scaling: float=0.2):
        if not self.debug_mode:
            return self.robot_mg.go_to_joint_state(pose, velocity_scaling=velocity_scaling, acc_scaling=acc_scaling)

    def _robot_open_gripper(self, wait):
        self.log("Opening gripper")
        if not self.disable_gripper:
            return self.robot_mg.open_gripper(wait=wait)

    def _robot_close_gripper(self, wait):
        self.log("Closing gripper")
        if not self.disable_gripper:
            return self.robot_mg.close_gripper(wait=wait)

    def _go_to_pose_cartesian_order(
        self,
        goal: Pose,
        acceleration_scaling_factor: float=0.2,
        velocity_scaling: float=0.2,
        reverse: bool = False,
    ) -> None:

        # go to required orientation
        current_pose = self.robot_mg.get_current_pose()
        if not self._robot_go_to_pose_goal(
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
            if not self._robot_go_to_pose_goal(
                offset_pose(self.robot_mg.get_current_pose(), offset),
                acc_scaling=acceleration_scaling_factor,
                velocity_scaling = velocity_scaling,
            ):
                return False
        return True

    def _robot_go_to_pose_goal(
        self, pose, acc_scaling=0.2, velocity_scaling=0.2, orient_tolerance=0.01
    ):
        if not self.debug_mode:
            return self.robot_mg.go_to_pose_goal(
                pose,
                acc_scaling=acc_scaling,
                velocity_scaling=velocity_scaling,
                orient_tolerance=orient_tolerance,
            )

    def _correct_gripper_angle_tilt(self, pose: Pose, reverse: bool = False) -> Pose:
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

class RecipeAction(ABC):
    guid: uuid.UUID = None

    def __init__(self) -> None:
        super().__init__()
        self.guid = uuid.uuid4()

class DispensingDelay(RecipeAction):
    duration: int = 0

    def __init__(self, _duration: int) -> None:
        super().__init__()
        self.duration = _duration

class DispensingRequest(RecipeAction):
    ingredient_id: int = None
    ingredient_name: str = None
    quantity: float = None

    def __init__(
        self,
        ingredient_id: int,
        ingredient_name: str,
        quantity: float,
        ingredient_pose: List[float],
    ) -> None:
        self.ingredient_id = ingredient_id
        self.ingredient_name = ingredient_name
        self.quantity = quantity
        self.ingredient_pose = ingredient_pose