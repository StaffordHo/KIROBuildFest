"""World domain model.

The World is the simulation environment containing robots,
objects, physics configuration, and simulation state.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import NewType, Optional

from .robot import Robot, RobotId
from .geometry import Vector3, Pose

WorldId = NewType("WorldId", str)


class SimStatus(Enum):
    """Simulation lifecycle states."""
    IDLE = "idle"            # Created but not started
    RUNNING = "running"      # Actively simulating
    PAUSED = "paused"        # Temporarily halted
    STOPPED = "stopped"      # Terminated
    ERROR = "error"          # Fatal error occurred


@dataclass(frozen=True)
class PhysicsConfig:
    """Physics engine configuration."""
    gravity: Vector3 = field(default_factory=lambda: Vector3(0, 0, -9.81))
    time_step: float = 1.0 / 240.0          # 240 Hz default
    solver_iterations: int = 50
    enable_collision: bool = True
    real_time_factor: float = 1.0            # 1.0 = real time, 2.0 = 2x speed


@dataclass
class StaticObject:
    """A non-movable object in the scene (ground plane, walls, etc.)."""
    name: str
    pose: Pose = field(default_factory=Pose.identity)
    geometry_type: str = "plane"  # plane, box, mesh
    dimensions: dict = field(default_factory=dict)
    color: tuple = (0.8, 0.8, 0.8, 1.0)


@dataclass
class DynamicObject:
    """A manipulable object in the scene."""
    name: str
    pose: Pose = field(default_factory=Pose.identity)
    geometry_type: str = "box"
    dimensions: dict = field(default_factory=dict)
    mass: float = 1.0
    color: tuple = (0.2, 0.6, 1.0, 1.0)


@dataclass
class SimulationState:
    """Mutable simulation runtime state."""
    status: SimStatus = SimStatus.IDLE
    sim_time: float = 0.0           # Elapsed simulation time (seconds)
    step_count: int = 0             # Total physics steps executed
    wall_time: float = 0.0          # Real elapsed time

    def advance(self, dt: float) -> None:
        """Advance simulation clock."""
        self.sim_time += dt
        self.step_count += 1


@dataclass
class World:
    """The simulation world — top-level aggregate.

    Contains all entities in the simulation: robots, objects,
    physics settings, and runtime state.
    """
    id: WorldId
    name: str
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    state: SimulationState = field(default_factory=SimulationState)
    robots: dict[str, Robot] = field(default_factory=dict)
    static_objects: list[StaticObject] = field(default_factory=list)
    dynamic_objects: list[DynamicObject] = field(default_factory=list)

    # --- Aggregate operations ---

    def add_robot(self, robot: Robot) -> None:
        """Add a robot to the world."""
        key = str(robot.id)
        if key in self.robots:
            raise ValueError(f"Robot with ID '{robot.id}' already exists in world")
        self.robots[key] = robot

    def remove_robot(self, robot_id: RobotId) -> None:
        """Remove a robot from the world."""
        key = str(robot_id)
        if key not in self.robots:
            raise ValueError(f"Robot '{robot_id}' not found in world")
        del self.robots[key]

    def add_static_object(self, obj: StaticObject) -> None:
        self.static_objects.append(obj)

    def add_dynamic_object(self, obj: DynamicObject) -> None:
        self.dynamic_objects.append(obj)

    # --- Simulation lifecycle ---

    def start(self) -> None:
        """Start or resume the simulation."""
        if self.state.status in (SimStatus.IDLE, SimStatus.PAUSED, SimStatus.STOPPED):
            self.state.status = SimStatus.RUNNING

    def pause(self) -> None:
        """Pause the simulation."""
        if self.state.status == SimStatus.RUNNING:
            self.state.status = SimStatus.PAUSED

    def stop(self) -> None:
        """Stop the simulation."""
        self.state.status = SimStatus.STOPPED

    def reset(self) -> None:
        """Reset simulation to initial state."""
        self.state = SimulationState()

    @property
    def is_running(self) -> bool:
        return self.state.status == SimStatus.RUNNING
