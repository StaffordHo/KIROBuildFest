"""Manipulation task domain models.

Defines graspable objects, manipulation goals, and pick-and-place
scenarios for validating robot control algorithms.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import NewType, Optional

from .geometry import Vector3, Quaternion, Pose, Box, Cylinder, Sphere

ManipulationTaskId = NewType("ManipulationTaskId", str)


class ObjectShape(Enum):
    """Shapes of manipulable objects."""
    BOX = "box"
    CYLINDER = "cylinder"
    SPHERE = "sphere"


class GraspState(Enum):
    """State of a grasp attempt."""
    IDLE = "idle"
    APPROACHING = "approaching"
    GRASPING = "grasping"
    HOLDING = "holding"
    PLACING = "placing"
    RELEASED = "released"
    FAILED = "failed"


class TaskStatus(Enum):
    """Status of a manipulation task."""
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class GraspableObject:
    """An object that can be picked up and manipulated."""
    name: str
    shape: ObjectShape
    pose: Pose = field(default_factory=Pose.identity)
    dimensions: Vector3 = field(default_factory=lambda: Vector3(0.05, 0.05, 0.05))
    mass: float = 0.5              # kg
    friction: float = 0.8          # Surface friction coefficient
    color: tuple = (0.2, 0.6, 1.0, 1.0)  # RGBA
    is_grasped: bool = False
    grasped_by: Optional[str] = None  # Robot ID that's holding it

    @property
    def grasp_width(self) -> float:
        """Minimum gripper opening needed to grasp this object."""
        if self.shape == ObjectShape.SPHERE:
            return self.dimensions.x  # diameter
        elif self.shape == ObjectShape.CYLINDER:
            return self.dimensions.x * 2  # diameter
        else:
            return min(self.dimensions.x, self.dimensions.y)

    def pick_up(self, robot_id: str) -> bool:
        """Attempt to grasp this object."""
        if self.is_grasped:
            return False
        self.is_grasped = True
        self.grasped_by = robot_id
        return True

    def release(self, new_pose: Optional[Pose] = None) -> None:
        """Release the object."""
        self.is_grasped = False
        self.grasped_by = None
        if new_pose:
            self.pose = new_pose


@dataclass
class PlacementZone:
    """A target zone where objects should be placed."""
    name: str
    pose: Pose = field(default_factory=Pose.identity)
    size: Vector3 = field(default_factory=lambda: Vector3(0.1, 0.1, 0.01))
    color: tuple = (0.2, 0.8, 0.2, 0.5)  # Semi-transparent green
    accepts_shapes: list[ObjectShape] = field(default_factory=lambda: list(ObjectShape))
    is_occupied: bool = False


@dataclass
class ManipulationGoal:
    """A single pick-and-place goal."""
    object_name: str
    target_zone: str
    required_orientation: Optional[Quaternion] = None  # None = any orientation OK
    tolerance_position: float = 0.02  # meters
    tolerance_angle: float = 0.1      # radians


@dataclass
class ManipulationTask:
    """A complete manipulation task with multiple goals.

    Represents a scenario like:
    - Pick up red block, place on target A
    - Pick up blue cylinder, place on target B
    - Sort objects by color into zones
    """
    id: ManipulationTaskId
    name: str
    description: str = ""
    goals: list[ManipulationGoal] = field(default_factory=list)
    objects: list[GraspableObject] = field(default_factory=list)
    placement_zones: list[PlacementZone] = field(default_factory=list)
    status: TaskStatus = TaskStatus.PENDING
    completed_goals: int = 0
    time_limit: float = 60.0       # seconds
    elapsed_time: float = 0.0

    @property
    def progress(self) -> float:
        """Completion percentage."""
        if not self.goals:
            return 0.0
        return self.completed_goals / len(self.goals) * 100.0

    @property
    def is_complete(self) -> bool:
        return self.completed_goals >= len(self.goals)

    def check_goal(self, goal_index: int) -> bool:
        """Check if a specific goal has been achieved."""
        if goal_index >= len(self.goals):
            return False

        goal = self.goals[goal_index]
        obj = next((o for o in self.objects if o.name == goal.object_name), None)
        zone = next((z for z in self.placement_zones if z.name == goal.target_zone), None)

        if not obj or not zone:
            return False

        # Check position within tolerance
        dx = obj.pose.position.x - zone.pose.position.x
        dy = obj.pose.position.y - zone.pose.position.y
        dz = obj.pose.position.z - zone.pose.position.z
        distance = (dx**2 + dy**2 + dz**2) ** 0.5

        return distance <= goal.tolerance_position and not obj.is_grasped

    def update(self, dt: float) -> None:
        """Update task state."""
        if self.status != TaskStatus.IN_PROGRESS:
            return

        self.elapsed_time += dt

        # Check all goals
        completed = sum(1 for i in range(len(self.goals)) if self.check_goal(i))
        self.completed_goals = completed

        if self.is_complete:
            self.status = TaskStatus.COMPLETED
        elif self.elapsed_time >= self.time_limit:
            self.status = TaskStatus.FAILED


# ============================================================================
# PRESET SCENARIOS
# ============================================================================

def create_pick_and_place_scenario() -> ManipulationTask:
    """Create a basic pick-and-place task with colored blocks."""
    objects = [
        GraspableObject(
            name="red_block",
            shape=ObjectShape.BOX,
            pose=Pose(position=Vector3(0.3, 0.1, 0.025)),
            dimensions=Vector3(0.05, 0.05, 0.05),
            color=(0.9, 0.2, 0.2, 1.0),
        ),
        GraspableObject(
            name="blue_cylinder",
            shape=ObjectShape.CYLINDER,
            pose=Pose(position=Vector3(0.2, -0.15, 0.03)),
            dimensions=Vector3(0.02, 0.02, 0.06),
            color=(0.2, 0.4, 0.9, 1.0),
        ),
        GraspableObject(
            name="green_sphere",
            shape=ObjectShape.SPHERE,
            pose=Pose(position=Vector3(0.35, -0.05, 0.025)),
            dimensions=Vector3(0.025, 0.025, 0.025),
            color=(0.2, 0.8, 0.3, 1.0),
        ),
    ]

    zones = [
        PlacementZone(
            name="zone_A",
            pose=Pose(position=Vector3(-0.2, 0.2, 0.005)),
            size=Vector3(0.08, 0.08, 0.01),
            color=(1.0, 0.5, 0.0, 0.4),
        ),
        PlacementZone(
            name="zone_B",
            pose=Pose(position=Vector3(-0.2, -0.2, 0.005)),
            size=Vector3(0.08, 0.08, 0.01),
            color=(0.5, 0.0, 1.0, 0.4),
        ),
    ]

    goals = [
        ManipulationGoal(object_name="red_block", target_zone="zone_A"),
        ManipulationGoal(object_name="blue_cylinder", target_zone="zone_B"),
    ]

    return ManipulationTask(
        id=ManipulationTaskId("pick_place_basic"),
        name="Basic Pick & Place",
        description="Pick up colored objects and place them in designated zones.",
        goals=goals,
        objects=objects,
        placement_zones=zones,
        time_limit=120.0,
    )


def create_sorting_scenario() -> ManipulationTask:
    """Create a sorting task — group objects by shape."""
    objects = []
    for i in range(3):
        objects.append(GraspableObject(
            name=f"box_{i}",
            shape=ObjectShape.BOX,
            pose=Pose(position=Vector3(0.2 + i * 0.08, 0.1 * (i % 2 - 0.5), 0.025)),
            dimensions=Vector3(0.04, 0.04, 0.04),
            color=(0.9, 0.3, 0.1, 1.0),
        ))
    for i in range(2):
        objects.append(GraspableObject(
            name=f"sphere_{i}",
            shape=ObjectShape.SPHERE,
            pose=Pose(position=Vector3(0.15 + i * 0.1, -0.15, 0.02)),
            dimensions=Vector3(0.02, 0.02, 0.02),
            color=(0.1, 0.5, 0.9, 1.0),
        ))

    zones = [
        PlacementZone(
            name="boxes_zone",
            pose=Pose(position=Vector3(-0.25, 0.15, 0.005)),
            size=Vector3(0.15, 0.15, 0.01),
            accepts_shapes=[ObjectShape.BOX],
        ),
        PlacementZone(
            name="spheres_zone",
            pose=Pose(position=Vector3(-0.25, -0.15, 0.005)),
            size=Vector3(0.15, 0.15, 0.01),
            accepts_shapes=[ObjectShape.SPHERE],
        ),
    ]

    goals = [ManipulationGoal(object_name=o.name, target_zone="boxes_zone" if o.shape == ObjectShape.BOX else "spheres_zone") for o in objects]

    return ManipulationTask(
        id=ManipulationTaskId("sort_by_shape"),
        name="Sort by Shape",
        description="Sort all objects into zones based on their shape.",
        goals=goals,
        objects=objects,
        placement_zones=zones,
        time_limit=180.0,
    )
