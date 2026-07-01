"""Simple pure-Python physics engine (fallback).

A lightweight kinematic simulation that doesn't require PyBullet.
Handles joint position control, basic gravity, and forward kinematics.
Sufficient for validating control algorithms and visualization.

For full dynamics (contacts, friction, inertia), use PyBulletEngine
on a Linux deployment or install Visual C++ build tools on Windows.
"""

from __future__ import annotations
import math
from typing import Optional

from ...domain.services.simulation_service import PhysicsEnginePort
from ...domain.models.world import World
from ...domain.models.robot import Robot
from ...domain.models.joint import JointState, JointType
from ...domain.models.geometry import Vector3, Quaternion


class SimplePhysicsEngine(PhysicsEnginePort):
    """Pure-Python kinematic physics engine.

    Simulates:
    - Joint position/velocity control with limits
    - Forward kinematics (link pose computation)
    - Basic damping

    Does NOT simulate:
    - Contact/collision dynamics
    - Rigid body inertia
    - Friction forces
    """

    def __init__(self):
        self._robots: dict[int, _RobotState] = {}
        self._next_id = 1
        self._gravity = Vector3(0, 0, -9.81)
        self._dt = 1 / 240.0

    def initialize(self, world: World) -> None:
        """Initialize with world config."""
        self._gravity = world.physics.gravity
        self._dt = world.physics.time_step
        self._robots.clear()
        self._next_id = 1

    def load_robot(self, robot: Robot) -> int:
        """Load robot — store joint info for simulation."""
        robot_id = self._next_id
        self._next_id += 1

        joint_states = {}
        joint_targets = {}
        joint_info = {}

        for name, joint in robot.joints.items():
            if joint.is_actuated:
                joint_states[name] = JointState(
                    position=joint.state.position,
                    velocity=0.0,
                    effort=0.0,
                )
                joint_targets[name] = joint.state.position
                joint_info[name] = {
                    "type": joint.joint_type,
                    "limits": joint.limits,
                    "damping": joint.damping,
                    "axis": joint.axis,
                    "origin": joint.origin,
                    "parent_link": joint.parent_link,
                    "child_link": joint.child_link,
                }

        self._robots[robot_id] = _RobotState(
            robot_id=str(robot.id),
            joint_states=joint_states,
            joint_targets=joint_targets,
            joint_info=joint_info,
            robot=robot,
        )

        return robot_id

    def load_robot_from_urdf(self, urdf_path: str, base_position: list = None) -> int:
        """Load from URDF — parse and load."""
        from .urdf_loader import URDFLoader
        loader = URDFLoader()
        robot = loader.load_from_file(urdf_path)
        return self.load_robot(robot)

    def step(self, dt: float) -> None:
        """Advance simulation — apply PD control to move joints toward targets."""
        for robot_state in self._robots.values():
            for joint_name, state in robot_state.joint_states.items():
                target = robot_state.joint_targets.get(joint_name, state.position)
                info = robot_state.joint_info[joint_name]
                limits = info["limits"]
                damping = info["damping"]

                # Simple PD controller
                kp = 50.0
                kd = 5.0 + damping

                error = target - state.position
                effort = kp * error - kd * state.velocity

                # Clamp effort
                max_effort = limits.effort
                effort = max(-max_effort, min(max_effort, effort))

                # Simple integration (semi-implicit Euler)
                # Assume unit inertia for simplicity
                acceleration = effort
                new_velocity = state.velocity + acceleration * dt
                new_velocity = max(-limits.velocity, min(limits.velocity, new_velocity))

                # Apply damping
                new_velocity *= (1.0 - damping * dt)

                new_position = state.position + new_velocity * dt

                # Enforce limits
                jtype = info["type"]
                if jtype in (JointType.REVOLUTE, JointType.PRISMATIC):
                    new_position = max(limits.lower, min(limits.upper, new_position))
                    if new_position == limits.lower or new_position == limits.upper:
                        new_velocity = 0.0

                state.position = new_position
                state.velocity = new_velocity
                state.effort = effort

    def get_joint_states(self, robot_id: int) -> dict[str, JointState]:
        """Get current joint states."""
        robot_state = self._robots.get(robot_id)
        if not robot_state:
            return {}
        # Return copies
        return {
            name: JointState(
                position=s.position,
                velocity=s.velocity,
                effort=s.effort,
            )
            for name, s in robot_state.joint_states.items()
        }

    def apply_joint_commands(self, robot_id: int, commands: dict[str, float]) -> None:
        """Apply effort commands (treated as targets in this simple engine)."""
        robot_state = self._robots.get(robot_id)
        if not robot_state:
            return
        # In simple mode, treat commands as position targets
        for joint_name, value in commands.items():
            if joint_name in robot_state.joint_targets:
                robot_state.joint_targets[joint_name] = value

    def set_joint_positions(self, robot_id: int, positions: dict[str, float]) -> None:
        """Set joint position targets."""
        robot_state = self._robots.get(robot_id)
        if not robot_state:
            return
        for joint_name, position in positions.items():
            if joint_name in robot_state.joint_targets:
                robot_state.joint_targets[joint_name] = position

    def get_link_pose(self, robot_id: int, link_name: str) -> tuple:
        """Compute link pose via forward kinematics."""
        robot_state = self._robots.get(robot_id)
        if not robot_state:
            return ([0, 0, 0], [0, 0, 0, 1])

        poses = self._compute_fk(robot_state)
        pose = poses.get(link_name, ([0, 0, 0], [0, 0, 0, 1]))
        return pose

    def get_all_link_poses(self, robot_id: int) -> dict[str, tuple]:
        """Get all link poses via forward kinematics."""
        robot_state = self._robots.get(robot_id)
        if not robot_state:
            return {}
        return self._compute_fk(robot_state)

    def reset(self) -> None:
        """Reset all joints to zero."""
        for robot_state in self._robots.values():
            for state in robot_state.joint_states.values():
                state.position = 0.0
                state.velocity = 0.0
                state.effort = 0.0
            for name in robot_state.joint_targets:
                robot_state.joint_targets[name] = 0.0

    def shutdown(self) -> None:
        """Clean up."""
        self._robots.clear()

    def _compute_fk(self, robot_state: _RobotState) -> dict[str, tuple]:
        """Compute forward kinematics for all links."""
        poses = {}
        robot = robot_state.robot

        # Find base link
        base = robot.base_link
        if not base:
            return poses

        base_pos = [
            robot.base_pose.position.x,
            robot.base_pose.position.y,
            robot.base_pose.position.z,
        ]
        base_orn = [
            robot.base_pose.orientation.x,
            robot.base_pose.orientation.y,
            robot.base_pose.orientation.z,
            robot.base_pose.orientation.w,
        ]
        poses[base.name] = (base_pos, base_orn)

        # BFS through kinematic tree
        processed = {base.name}
        queue = [base.name]

        while queue:
            parent_name = queue.pop(0)
            parent_pos, parent_orn = poses[parent_name]

            for joint_name, info in robot_state.joint_info.items():
                if info["parent_link"] == parent_name and info["child_link"] not in processed:
                    child_name = info["child_link"]
                    origin = info["origin"]

                    # Get joint position
                    q = robot_state.joint_states.get(joint_name)
                    q_val = q.position if q else 0.0

                    # Compute child position (simplified)
                    ox = origin.position.x
                    oy = origin.position.y
                    oz = origin.position.z

                    # For revolute joints, rotate the offset
                    axis = info["axis"]
                    jtype = info["type"]

                    if jtype in (JointType.REVOLUTE, JointType.CONTINUOUS):
                        # Simplified rotation around axis
                        cos_q = math.cos(q_val)
                        sin_q = math.sin(q_val)

                        # Apply rotation to offset based on axis
                        if abs(axis.z) > 0.5:  # Z-axis rotation
                            rx = ox * cos_q - oy * sin_q
                            ry = ox * sin_q + oy * cos_q
                            rz = oz
                        elif abs(axis.y) > 0.5:  # Y-axis rotation
                            rx = ox * cos_q + oz * sin_q
                            ry = oy
                            rz = -ox * sin_q + oz * cos_q
                        else:  # X-axis rotation
                            rx = ox
                            ry = oy * cos_q - oz * sin_q
                            rz = oy * sin_q + oz * cos_q

                        child_pos = [
                            parent_pos[0] + rx,
                            parent_pos[1] + ry,
                            parent_pos[2] + rz,
                        ]
                        # Simplified orientation
                        child_orn = Quaternion.from_euler(
                            q_val * axis.x, q_val * axis.y, q_val * axis.z
                        )
                        child_orn_list = [child_orn.x, child_orn.y, child_orn.z, child_orn.w]

                    elif jtype == JointType.PRISMATIC:
                        child_pos = [
                            parent_pos[0] + ox + q_val * axis.x,
                            parent_pos[1] + oy + q_val * axis.y,
                            parent_pos[2] + oz + q_val * axis.z,
                        ]
                        child_orn_list = parent_orn

                    else:
                        child_pos = [
                            parent_pos[0] + ox,
                            parent_pos[1] + oy,
                            parent_pos[2] + oz,
                        ]
                        child_orn_list = parent_orn

                    poses[child_name] = (child_pos, child_orn_list)
                    processed.add(child_name)
                    queue.append(child_name)

        return poses


class _RobotState:
    """Internal state tracking for a loaded robot."""

    def __init__(
        self,
        robot_id: str,
        joint_states: dict[str, JointState],
        joint_targets: dict[str, float],
        joint_info: dict,
        robot: Robot,
    ):
        self.robot_id = robot_id
        self.joint_states = joint_states
        self.joint_targets = joint_targets
        self.joint_info = joint_info
        self.robot = robot
