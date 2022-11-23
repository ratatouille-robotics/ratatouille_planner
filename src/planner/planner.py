from abc import ABC, abstractmethod
import uuid
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
_KNOWN_POSES_FILE_PATH = "poses.yaml"

_CONTAINER_POSITION_START = 1
_CONTAINER_POSITION_END = 15


class IngredientTypes(str, Enum):
    OLIVES_BLACK = "blackolives"
    BLACK_PEPPER = "black_pepper"
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
    ONION = "onion"
    NO_INGREDIENT = "no_ingredient"
    NO_CONTAINER = "no_container"
    SUNFLOWER_OIL = "sunflower_oil"
    GARLIC_POWDER = "garlic_powder"
    MARINARA = "marinara"
    PASTA = "pasta"
    CHEESE = "cheese"
    BELL_PEPPER = "bell_pepper"

    def __str__(self) -> str:
        return str.__str__(self)

    def __repr__(self) -> str:
        return str.__repr__(self)


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


class InventoryItem(yaml.YAMLObject):
    position: int = None
    name: str = None
    quantity: float = None
    pose: List[float] = None

    def __init__(self, position: int, name: str, quantity: float, pose: List[float]):
        self.position = position
        self.name = name
        self.quantity = quantity
        self.pose = pose

    def __repr__(self) -> str:
        return f"position: {self.position}, name: {self.name}, quantity: {self.quantity}, pose: {self.pose}"


def inventoryitem_constructor(
    loader: yaml.SafeLoader, node: yaml.nodes.MappingNode
) -> InventoryItem:
    """Construct an inventory item."""
    return InventoryItem(**loader.construct_mapping(node))


class Ingredient(yaml.YAMLObject):
    name: str = None
    quantity: float = None

    def __init__(self, name: str, quantity: float):
        self.name = name
        self.quantity = quantity

    def __repr__(self) -> str:
        return f"name: {self.name}, quantity: {self.quantity}"


def ingredient_constructor(
    loader: yaml.SafeLoader, node: yaml.nodes.MappingNode
) -> Ingredient:
    """Construct an ingredient."""
    return Ingredient(**loader.construct_mapping(node))


class Recipe(yaml.YAMLObject):
    name: str = None
    id: int = None
    ingredients: List[Ingredient] = None

    def __init__(self, id: int, name: str, ingredients: List[Ingredient]):
        self.id = id
        self.name = name
        self.ingredients = ingredients

    def __repr__(self) -> str:
        return f"name: {self.name}, id: {self.id}, ingredients: {self.ingredients}"


def recipe_constructor(loader: yaml.SafeLoader, node: yaml.nodes.MappingNode) -> Recipe:
    """Construct a recipe."""
    return Recipe(**loader.construct_mapping(node))


def get_yaml_loader():
    """Add constructors to PyYAML loader."""
    loader = yaml.SafeLoader
    loader.add_constructor("!ingredient", ingredient_constructor)
    loader.add_constructor("!recipe", recipe_constructor)
    loader.add_constructor("!inventoryitem", inventoryitem_constructor)
    return loader


def ingredient_representer(
    dumper: yaml.SafeDumper, item: Ingredient
) -> yaml.nodes.MappingNode:
    """Represent an ingredient instance as a YAML mapping node."""
    return dumper.represent_mapping(
        "!ingredient",
        {
            "name": item.name,
            "quantity": item.quantity,
        },
    )


def recipe_representer(
    dumper: yaml.SafeDumper, recipe: Recipe
) -> yaml.nodes.MappingNode:
    """Represent a recipe instance as a YAML mapping node."""
    return dumper.represent_mapping(
        "!recipe",
        {
            "id": recipe.id,
            "name": recipe.name,
            "ingredients": recipe.ingredients,
        },
    )


def inventoryitem_representer(
    dumper: yaml.SafeDumper, item: InventoryItem
) -> yaml.nodes.MappingNode:
    """Represent an inventory item instance as a YAML mapping node."""
    return dumper.represent_mapping(
        "!inventoryitem",
        {
            "position": item.position,
            "name": item.name,
            "pose": item.pose,
            "quantity": item.quantity,
        },
    )


def get_yaml_dumper():
    """Add representers to a YAML seriailizer."""
    safe_dumper = yaml.SafeDumper
    safe_dumper.add_representer(Ingredient, ingredient_representer)
    safe_dumper.add_representer(Recipe, recipe_representer)
    safe_dumper.add_representer(InventoryItem, inventoryitem_representer)
    return safe_dumper


class RatatouillePlanner(ABC):
    debug_mode = False
    disable_gripper = False
    verbose = True
    config_dir_path = None

    inventory = None

    pose_transformer = None
    robot_mg = None
    known_poses = None
    recipes: List[Recipe] = None
    inventory: Dict[int, InventoryItem] = None

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
            file=os.path.join(self.config_dir_path, _KNOWN_POSES_FILE_PATH),
            mode="r",
        ) as _temp:
            self.known_poses = yaml.load(_temp, Loader=yaml.SafeLoader)

    def load_recipes(self):
        with open(
            file=os.path.join(self.config_dir_path, _RECIPE_FILE_PATH),
            mode="r",
        ) as _temp:
            self.recipes = yaml.load(_temp, Loader=get_yaml_loader())

    def load_inventory(self):
        # initialize self.inventory with None at each position
        # cannot use defaultdict since we need fixed length of positions
        # from first container (1) to last last container (12)
        self.inventory = {
            key: None
            for key in range(_CONTAINER_POSITION_START, _CONTAINER_POSITION_END + 1)
        }
        filename = os.path.join(self.config_dir_path, _INVENTORY_FILE_PATH)
        # create inventory file if not exists
        if not os.path.exists(filename):
            with open(file=filename, mode="w"):
                pass
        # load inventory
        with open(
            file=filename,
            mode="r",
        ) as _temp:
            _inventory = yaml.load(_temp, Loader=get_yaml_loader())
            if not _inventory:
                return
            for item in _inventory:
                self.inventory[item.position] = item

    def get_position_of_ingredient(self, ingredient_name: str):
        for item in self.inventory.values():
            if item.name == ingredient_name:
                return item.position
        return None

    def write_inventory(self) -> None:
        # skip writing null values
        _inventory = [x for x in self.inventory.values() if x]
        with open(
            os.path.join(self.config_dir_path, _INVENTORY_FILE_PATH), "w"
        ) as _temp:
            yaml.dump(_inventory, _temp, Dumper=get_yaml_dumper())

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

    def _robot_go_to_joint_state(
        self, pose, velocity_scaling: float = 0.2, acc_scaling: float = 0.2
    ):
        if not self.debug_mode:
            return self.robot_mg.go_to_joint_state(
                pose, velocity_scaling=velocity_scaling, acc_scaling=acc_scaling
            )

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
        acceleration_scaling_factor: float = 0.2,
        velocity_scaling: float = 0.2,
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
                velocity_scaling=velocity_scaling,
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

    def get_recipe_by_id(self, recipe_id: int) -> dict:
        for recipe in self.recipes:
            if recipe.id == recipe_id:
                print(recipe)
                return recipe
        return None


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
        super().__init__()
        self.ingredient_id = ingredient_id
        self.ingredient_name = ingredient_name
        self.quantity = quantity
        self.ingredient_pose = ingredient_pose
