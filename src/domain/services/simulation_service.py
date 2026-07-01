"""Simulation domain service.

Coordinates the simulation loop: physics stepping, sensor updates,
actuator commands. This service defines the contract — infrastructure
provides the implementation.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Optional

from ..models.world import World, SimStatus
from ..models.robot import Robot, RobotId
from ..models.joint import JointState
from ..models.sensor import SensorReading


class PhysicsEnginePort(ABC):
    """Port (interface) for physics engine — implemented by infrastructure."""

    @abstractmethod
    def initialize(self, world: World) -> None:
        """Initialize the physics engine with world configuration."""
        ...

    @abstractmethod
    def load_robot(self, robot: Robot) -> int:
        """Load a robot into the physics engine. Returns engine-specific ID."""
        ...

    @abstractmethod
    def step(self, dt: float) -> None:
        """Advance physics by one time step."""
        ...

    @abstractmethod
    def get_joint_states(self, robot_id: int) -> dict[str, JointState]:
        """Read current joint states from physics engine."""
        ...

    @abstractmethod
    def apply_joint_commands(self, robot_id: int, commands: dict[str, float]) -> None:
        """Apply joint position/velocity/effort commands."""
        ...

    @abstractmethod
    def get_link_pose(self, robot_id: int, link_name: str) -> tuple:
        """Get world-frame pose of a link. Returns (position, orientation)."""
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset the physics engine state."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Clean up physics engine resources."""
        ...


class SimulationService:
    """Orchestrates the simulation loop.

    This is a domain service that coordinates between the World aggregate
    and the physics engine (via port/adapter pattern).
    """

    def __init__(self, physics_engine: PhysicsEnginePort):
        self._physics = physics_engine
        self._robot_physics_ids: dict[str, int] = {}  # RobotId -> physics engine ID

    def initialize_world(self, world: World) -> None:
        """Set up the physics engine for a world."""
        self._physics.initialize(world)
        for robot_key, robot in world.robots.items():
            physics_id = self._physics.load_robot(robot)
            self._robot_physics_ids[robot_key] = physics_id

    def step(self, world: World) -> dict[str, dict[str, JointState]]:
        """Execute one simulation step.

        1. Apply actuator commands
        2. Step physics
        3. Read back joint states
        4. Update domain model

        Returns:
            Updated joint states per robot.
        """
        if not world.is_running:
            return {}

        dt = world.physics.time_step

        # Apply actuator commands to physics engine
        for robot_key, robot in world.robots.items():
            physics_id = self._robot_physics_ids.get(robot_key)
            if physics_id is None:
                continue

            commands = {}
            for actuator in robot.actuators.values():
                joint = robot.joints.get(actuator.joint_name)
                if joint:
                    effort = actuator.compute_effort(
                        joint.state.position,
                        joint.state.velocity,
                        dt,
                    )
                    commands[actuator.joint_name] = effort

            if commands:
                self._physics.apply_joint_commands(physics_id, commands)

        # Step physics
        self._physics.step(dt)

        # Read back state and update domain model
        all_states = {}
        for robot_key, robot in world.robots.items():
            physics_id = self._robot_physics_ids.get(robot_key)
            if physics_id is None:
                continue

            joint_states = self._physics.get_joint_states(physics_id)
            for joint_name, state in joint_states.items():
                if joint_name in robot.joints:
                    robot.joints[joint_name].state = state

            all_states[robot_key] = joint_states

        # Advance simulation clock
        world.state.advance(dt)
        return all_states

    def reset(self, world: World) -> None:
        """Reset simulation to initial state."""
        self._physics.reset()
        world.reset()
        self._robot_physics_ids.clear()

    def shutdown(self) -> None:
        """Clean up resources."""
        self._physics.shutdown()
