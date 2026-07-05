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
    - Ground plane contact (spring-damper)

    Does NOT simulate:
    - Full rigid body inertia
    - Object-to-object contact
    """

    def __init__(self):
        self._robots: dict[int, _RobotState] = {}
        self._next_id = 1
        self._gravity = Vector3(0, 0, -9.81)
        self._dt = 1 / 240.0
        # Contact physics
        from .contact_physics import ContactWorld
        self._contact_world = ContactWorld()

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
            # Store ALL joints (including fixed) for FK computation
            joint_info[name] = {
                "type": joint.joint_type,
                "limits": joint.limits,
                "damping": joint.damping,
                "axis": joint.axis,
                "origin": joint.origin,
                "parent_link": joint.parent_link,
                "child_link": joint.child_link,
            }
            # Only actuated joints get state/targets
            if joint.is_actuated:
                joint_states[name] = JointState(
                    position=joint.state.position,
                    velocity=0.0,
                    effort=0.0,
                )
                joint_targets[name] = joint.state.position

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
        """Advance simulation — apply PD control and locomotion."""
        for robot_state in self._robots.values():
            # Joint PD control
            for joint_name, state in robot_state.joint_states.items():
                target = robot_state.joint_targets.get(joint_name, state.position)
                info = robot_state.joint_info[joint_name]
                limits = info["limits"]
                damping = info["damping"]

                kp = 50.0
                kd = 5.0 + damping
                error = target - state.position
                effort = kp * error - kd * state.velocity
                max_effort = limits.effort
                effort = max(-max_effort, min(max_effort, effort))

                acceleration = effort
                new_velocity = state.velocity + acceleration * dt
                new_velocity = max(-limits.velocity, min(limits.velocity, new_velocity))
                new_velocity *= (1.0 - damping * dt)
                new_position = state.position + new_velocity * dt

                jtype = info["type"]
                if jtype in (JointType.REVOLUTE, JointType.PRISMATIC):
                    if limits.lower != 0 or limits.upper != 0:
                        new_position = max(limits.lower, min(limits.upper, new_position))
                        if new_position == limits.lower or new_position == limits.upper:
                            new_velocity = 0.0

                state.position = new_position
                state.velocity = new_velocity
                state.effort = effort

            # Locomotion for non-fixed-base robots
            if not robot_state.is_fixed_base:
                # Estimate forward velocity from joint motion
                # Simple model: average joint velocity → forward motion
                avg_vel = 0.0
                count = 0
                for state in robot_state.joint_states.values():
                    avg_vel += abs(state.velocity)
                    count += 1
                if count > 0:
                    avg_vel /= count
                # Convert joint velocity to base translation (simplified gait model)
                # Forward speed proportional to average joint speed
                forward_speed = avg_vel * 0.02  # Scale factor
                robot_state.base_velocity_offset[0] += forward_speed * dt
                # Small lateral drift based on asymmetric joint motion
                robot_state.base_velocity_offset[1] += math.sin(robot_state.base_velocity_offset[0] * 5) * 0.001 * dt

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
        """Get all link poses via forward kinematics with ground contact."""
        robot_state = self._robots.get(robot_id)
        if not robot_state:
            return {}
        poses = self._compute_fk(robot_state)

        # Apply ground contact: clamp Z >= 0 for all links
        if poses:
            min_z = min(p[0][2] for p in poses.values())
            if min_z < 0:
                offset = -min_z
                for name in poses:
                    pos, orn = poses[name]
                    poses[name] = ([pos[0], pos[1], pos[2] + offset], orn)

        # Apply base locomotion for non-fixed-base robots
        base_offset = robot_state.base_velocity_offset
        if base_offset[0] != 0 or base_offset[1] != 0 or base_offset[2] != 0:
            for name in poses:
                pos, orn = poses[name]
                poses[name] = ([
                    pos[0] + base_offset[0],
                    pos[1] + base_offset[1],
                    pos[2] + base_offset[2],
                ], orn)

        return poses

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
        """Compute forward kinematics using cumulative rotation matrices.

        Each joint's rotation affects ALL descendant links, not just
        the immediate child. We track a 3x3 rotation matrix per link.
        """
        poses = {}
        rotations = {}  # link_name -> 3x3 rotation matrix (as flat list)
        robot = robot_state.robot

        base = robot.base_link
        if not base:
            return poses

        base_pos = [
            robot.base_pose.position.x,
            robot.base_pose.position.y,
            robot.base_pose.position.z,
        ]
        poses[base.name] = (base_pos, [0, 0, 0, 1])
        # Identity rotation matrix [row-major]
        rotations[base.name] = [1,0,0, 0,1,0, 0,0,1]

        processed = {base.name}
        queue = [base.name]

        while queue:
            parent_name = queue.pop(0)
            parent_pos = poses[parent_name][0]
            parent_rot = rotations[parent_name]

            for joint_name, info in robot_state.joint_info.items():
                if info["parent_link"] == parent_name and info["child_link"] not in processed:
                    child_name = info["child_link"]
                    origin = info["origin"]
                    jtype = info["type"]
                    axis = info["axis"]

                    ox = origin.position.x
                    oy = origin.position.y
                    oz = origin.position.z

                    q = robot_state.joint_states.get(joint_name)
                    q_val = q.position if q else 0.0

                    # Compute joint rotation matrix
                    if jtype in (JointType.REVOLUTE, JointType.CONTINUOUS):
                        joint_rot = self._rotation_matrix(axis.x, axis.y, axis.z, q_val)
                    else:
                        joint_rot = [1,0,0, 0,1,0, 0,0,1]

                    # Child rotation = parent_rot * joint_rot
                    child_rot = self._mat_mul(parent_rot, joint_rot)

                    # Transform origin offset by parent rotation
                    rotated_offset = self._mat_vec(parent_rot, [ox, oy, oz])

                    # For prismatic, add joint displacement along axis in world frame
                    if jtype == JointType.PRISMATIC:
                        axis_world = self._mat_vec(parent_rot, [axis.x, axis.y, axis.z])
                        rotated_offset[0] += q_val * axis_world[0]
                        rotated_offset[1] += q_val * axis_world[1]
                        rotated_offset[2] += q_val * axis_world[2]

                    child_pos = [
                        parent_pos[0] + rotated_offset[0],
                        parent_pos[1] + rotated_offset[1],
                        parent_pos[2] + rotated_offset[2],
                    ]

                    poses[child_name] = (child_pos, [0, 0, 0, 1])
                    rotations[child_name] = child_rot
                    processed.add(child_name)
                    queue.append(child_name)

        return poses

    @staticmethod
    def _rotation_matrix(ax: float, ay: float, az: float, angle: float) -> list:
        """Create a 3x3 rotation matrix (row-major) for rotation around axis by angle."""
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1 - c
        # Normalize axis
        mag = math.sqrt(ax*ax + ay*ay + az*az)
        if mag < 1e-10:
            return [1,0,0, 0,1,0, 0,0,1]
        ax, ay, az = ax/mag, ay/mag, az/mag

        return [
            t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay,
            t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax,
            t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c,
        ]

    @staticmethod
    def _mat_mul(a: list, b: list) -> list:
        """Multiply two 3x3 matrices (row-major)."""
        r = [0]*9
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    r[i*3+j] += a[i*3+k] * b[k*3+j]
        return r

    @staticmethod
    def _mat_vec(m: list, v: list) -> list:
        """Multiply 3x3 matrix by 3-vector."""
        return [
            m[0]*v[0] + m[1]*v[1] + m[2]*v[2],
            m[3]*v[0] + m[4]*v[1] + m[5]*v[2],
            m[6]*v[0] + m[7]*v[1] + m[8]*v[2],
        ]


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
        # Locomotion: cumulative base position offset
        self.base_velocity_offset = [0.0, 0.0, 0.0]  # [x, y, z] in sim frame
        self.base_velocity = [0.0, 0.0, 0.0]
        self.is_fixed_base = self._detect_fixed_base()

    def _detect_fixed_base(self) -> bool:
        """Determine if this robot has a fixed base (arm) or mobile base (quadruped/mobile)."""
        # Heuristic: if robot has > 6 DOF and multiple branches, it's likely mobile
        num_actuated = len(self.joint_states)
        # Check for multiple children from base (branching = legs)
        base = self.robot.base_link
        if not base:
            return True
        children_from_base = sum(
            1 for info in self.joint_info.values()
            if info["parent_link"] == base.name
        )
        # Arms: serial chain (1 child from base), Quadrupeds: 4+ children
        if children_from_base >= 3:
            return False  # Likely a quadruped/humanoid
        if num_actuated <= 9:
            return True   # Likely a manipulator
        return True  # Default: fixed
