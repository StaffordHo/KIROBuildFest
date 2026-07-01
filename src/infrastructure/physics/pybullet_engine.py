"""PyBullet physics engine adapter.

Implements the PhysicsEnginePort interface using PyBullet (Bullet Physics SDK).
This is the primary physics backend for RoboSim.
"""

from __future__ import annotations
import pybullet as p
import pybullet_data
import tempfile
import os
from pathlib import Path
from typing import Optional

from ...domain.services.simulation_service import PhysicsEnginePort
from ...domain.models.world import World
from ...domain.models.robot import Robot
from ...domain.models.joint import JointState, JointType
from ...domain.models.geometry import Vector3


class PyBulletEngine(PhysicsEnginePort):
    """PyBullet-based physics engine implementation.

    Runs in DIRECT mode (no GUI) for server-side simulation.
    State is streamed to the frontend via WebSocket.
    """

    def __init__(self):
        self._physics_client: Optional[int] = None
        self._robot_bodies: dict[int, dict] = {}  # body_id -> joint info
        self._initialized = False

    def initialize(self, world: World) -> None:
        """Initialize PyBullet with world physics config."""
        if self._physics_client is not None:
            p.disconnect(self._physics_client)

        self._physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Apply physics config
        gravity = world.physics.gravity
        p.setGravity(gravity.x, gravity.y, gravity.z, physicsClientId=self._physics_client)
        p.setTimeStep(world.physics.time_step, physicsClientId=self._physics_client)
        p.setPhysicsEngineParameter(
            numSolverIterations=world.physics.solver_iterations,
            physicsClientId=self._physics_client,
        )

        # Add ground plane
        p.loadURDF("plane.urdf", physicsClientId=self._physics_client)

        # Load static objects
        for obj in world.static_objects:
            self._load_static_object(obj)

        # Load dynamic objects
        for obj in world.dynamic_objects:
            self._load_dynamic_object(obj)

        self._initialized = True

    def load_robot(self, robot: Robot) -> int:
        """Load a robot into PyBullet from its domain model.

        Generates a temporary URDF file from the domain model and loads it.
        Returns the PyBullet body ID.
        """
        urdf_content = self._generate_urdf(robot)

        # Write URDF to temp file
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf", delete=False, prefix=f"robosim_{robot.metadata.name}_"
        ) as f:
            f.write(urdf_content)
            urdf_path = f.name

        try:
            # Load into PyBullet
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

            body_id = p.loadURDF(
                urdf_path,
                basePosition=base_pos,
                baseOrientation=base_orn,
                useFixedBase=True,
                physicsClientId=self._physics_client,
            )

            # Map joint names to PyBullet joint indices
            joint_map = {}
            num_joints = p.getNumJoints(body_id, physicsClientId=self._physics_client)
            for i in range(num_joints):
                info = p.getJointInfo(body_id, i, physicsClientId=self._physics_client)
                joint_name = info[1].decode("utf-8")
                joint_map[joint_name] = i

            self._robot_bodies[body_id] = {
                "joint_map": joint_map,
                "robot_id": str(robot.id),
            }

            return body_id

        finally:
            os.unlink(urdf_path)

    def load_robot_from_urdf(self, urdf_path: str, base_position: list = None) -> int:
        """Load a robot directly from a URDF file path.

        Useful for importing open-source models without conversion.
        """
        if not self._initialized:
            raise RuntimeError("Physics engine not initialized")

        base_pos = base_position or [0, 0, 0]
        body_id = p.loadURDF(
            urdf_path,
            basePosition=base_pos,
            useFixedBase=True,
            physicsClientId=self._physics_client,
        )

        # Build joint map
        joint_map = {}
        num_joints = p.getNumJoints(body_id, physicsClientId=self._physics_client)
        for i in range(num_joints):
            info = p.getJointInfo(body_id, i, physicsClientId=self._physics_client)
            joint_name = info[1].decode("utf-8")
            joint_map[joint_name] = i

        self._robot_bodies[body_id] = {
            "joint_map": joint_map,
            "robot_id": f"urdf_{body_id}",
        }

        return body_id

    def step(self, dt: float) -> None:
        """Advance physics by one time step."""
        if self._physics_client is not None:
            p.stepSimulation(physicsClientId=self._physics_client)

    def get_joint_states(self, robot_id: int) -> dict[str, JointState]:
        """Read joint states from PyBullet."""
        states = {}
        body_info = self._robot_bodies.get(robot_id)
        if not body_info:
            return states

        for joint_name, joint_idx in body_info["joint_map"].items():
            js = p.getJointState(robot_id, joint_idx, physicsClientId=self._physics_client)
            states[joint_name] = JointState(
                position=js[0],
                velocity=js[1],
                effort=js[3],
            )

        return states

    def apply_joint_commands(self, robot_id: int, commands: dict[str, float]) -> None:
        """Apply force/torque commands to joints."""
        body_info = self._robot_bodies.get(robot_id)
        if not body_info:
            return

        for joint_name, effort in commands.items():
            joint_idx = body_info["joint_map"].get(joint_name)
            if joint_idx is not None:
                p.setJointMotorControl2(
                    bodyUniqueId=robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.TORQUE_CONTROL,
                    force=effort,
                    physicsClientId=self._physics_client,
                )

    def set_joint_positions(self, robot_id: int, positions: dict[str, float]) -> None:
        """Directly set joint positions (for position control mode)."""
        body_info = self._robot_bodies.get(robot_id)
        if not body_info:
            return

        for joint_name, position in positions.items():
            joint_idx = body_info["joint_map"].get(joint_name)
            if joint_idx is not None:
                p.setJointMotorControl2(
                    bodyUniqueId=robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=position,
                    force=100.0,
                    physicsClientId=self._physics_client,
                )

    def get_link_pose(self, robot_id: int, link_name: str) -> tuple:
        """Get world-frame pose of a link."""
        body_info = self._robot_bodies.get(robot_id)
        if not body_info:
            return ([0, 0, 0], [0, 0, 0, 1])

        # Find link index (links are indexed like joints in PyBullet)
        joint_map = body_info["joint_map"]
        # Check if any joint has this as child link
        for joint_name, joint_idx in joint_map.items():
            info = p.getJointInfo(robot_id, joint_idx, physicsClientId=self._physics_client)
            child_link_name = info[12].decode("utf-8")
            if child_link_name == link_name:
                state = p.getLinkState(robot_id, joint_idx, physicsClientId=self._physics_client)
                return (list(state[0]), list(state[1]))

        # Base link
        pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=self._physics_client)
        return (list(pos), list(orn))

    def get_all_link_poses(self, robot_id: int) -> dict[str, tuple]:
        """Get poses of all links for visualization streaming."""
        poses = {}
        body_info = self._robot_bodies.get(robot_id)
        if not body_info:
            return poses

        # Base link
        pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=self._physics_client)
        poses["base_link"] = (list(pos), list(orn))

        # All other links
        num_joints = p.getNumJoints(robot_id, physicsClientId=self._physics_client)
        for i in range(num_joints):
            info = p.getJointInfo(robot_id, i, physicsClientId=self._physics_client)
            link_name = info[12].decode("utf-8")
            state = p.getLinkState(robot_id, i, physicsClientId=self._physics_client)
            poses[link_name] = (list(state[0]), list(state[1]))

        return poses

    def reset(self) -> None:
        """Reset physics engine."""
        if self._physics_client is not None:
            p.resetSimulation(physicsClientId=self._physics_client)
            self._robot_bodies.clear()

    def shutdown(self) -> None:
        """Disconnect from PyBullet."""
        if self._physics_client is not None:
            p.disconnect(self._physics_client)
            self._physics_client = None
            self._robot_bodies.clear()
            self._initialized = False

    # --- Private helpers ---

    def _generate_urdf(self, robot: Robot) -> str:
        """Generate URDF XML from domain model."""
        lines = [
            '<?xml version="1.0"?>',
            f'<robot name="{robot.metadata.name}">',
        ]

        for link in robot.links.values():
            lines.append(f'  <link name="{link.name}">')
            # Inertial
            lines.append("    <inertial>")
            lines.append(f"      <mass value=\"{link.inertial.mass}\"/>")
            lines.append(
                f'      <inertia ixx="{link.inertial.ixx}" ixy="{link.inertial.ixy}" '
                f'ixz="{link.inertial.ixz}" iyy="{link.inertial.iyy}" '
                f'iyz="{link.inertial.iyz}" izz="{link.inertial.izz}"/>'
            )
            lines.append("    </inertial>")
            # Visual
            for visual in link.visuals:
                lines.append("    <visual>")
                if visual.geometry:
                    lines.append("      <geometry>")
                    lines.append(self._geometry_to_urdf(visual.geometry))
                    lines.append("      </geometry>")
                if visual.color:
                    r, g, b, a = visual.color
                    lines.append(f'      <material name=""><diffuse>{r} {g} {b} {a}</diffuse></material>')
                lines.append("    </visual>")
            # Collision
            for col in link.collisions:
                lines.append("    <collision>")
                if col.geometry:
                    lines.append("      <geometry>")
                    lines.append(self._geometry_to_urdf(col.geometry))
                    lines.append("      </geometry>")
                lines.append("    </collision>")
            lines.append("  </link>")

        for joint in robot.joints.values():
            jtype = joint.joint_type.value
            lines.append(f'  <joint name="{joint.name}" type="{jtype}">')
            lines.append(f'    <parent link="{joint.parent_link}"/>')
            lines.append(f'    <child link="{joint.child_link}"/>')
            o = joint.origin.position
            lines.append(f'    <origin xyz="{o.x} {o.y} {o.z}"/>')
            a = joint.axis
            lines.append(f'    <axis xyz="{a.x} {a.y} {a.z}"/>')
            if joint.has_limits:
                lim = joint.limits
                lines.append(
                    f'    <limit lower="{lim.lower}" upper="{lim.upper}" '
                    f'velocity="{lim.velocity}" effort="{lim.effort}"/>'
                )
            lines.append("  </joint>")

        lines.append("</robot>")
        return "\n".join(lines)

    def _geometry_to_urdf(self, geom) -> str:
        """Convert a geometry value object to URDF XML."""
        from ...domain.models.geometry import Box, Cylinder, Sphere, Mesh

        if isinstance(geom, Box):
            return f'        <box size="{geom.size_x} {geom.size_y} {geom.size_z}"/>'
        elif isinstance(geom, Cylinder):
            return f'        <cylinder radius="{geom.radius}" length="{geom.length}"/>'
        elif isinstance(geom, Sphere):
            return f'        <sphere radius="{geom.radius}"/>'
        elif isinstance(geom, Mesh):
            return f'        <mesh filename="{geom.filename}"/>'
        return '        <sphere radius="0.01"/>'

    def _load_static_object(self, obj) -> None:
        """Load a static object into PyBullet."""
        if obj.geometry_type == "plane":
            # Already loaded as ground plane
            pass
        elif obj.geometry_type == "box":
            dims = obj.dimensions
            half_extents = [
                dims.get("x", 0.5) / 2,
                dims.get("y", 0.5) / 2,
                dims.get("z", 0.5) / 2,
            ]
            col_id = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=half_extents,
                physicsClientId=self._physics_client,
            )
            vis_id = p.createVisualShape(
                p.GEOM_BOX, halfExtents=half_extents,
                rgbaColor=obj.color,
                physicsClientId=self._physics_client,
            )
            pos = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
            p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=col_id,
                baseVisualShapeIndex=vis_id, basePosition=pos,
                physicsClientId=self._physics_client,
            )

    def _load_dynamic_object(self, obj) -> None:
        """Load a dynamic object into PyBullet."""
        dims = obj.dimensions
        if obj.geometry_type == "box":
            half_extents = [
                dims.get("x", 0.05) / 2,
                dims.get("y", 0.05) / 2,
                dims.get("z", 0.05) / 2,
            ]
            col_id = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=half_extents,
                physicsClientId=self._physics_client,
            )
            vis_id = p.createVisualShape(
                p.GEOM_BOX, halfExtents=half_extents,
                rgbaColor=obj.color,
                physicsClientId=self._physics_client,
            )
        elif obj.geometry_type == "sphere":
            radius = dims.get("radius", 0.05)
            col_id = p.createCollisionShape(
                p.GEOM_SPHERE, radius=radius,
                physicsClientId=self._physics_client,
            )
            vis_id = p.createVisualShape(
                p.GEOM_SPHERE, radius=radius,
                rgbaColor=obj.color,
                physicsClientId=self._physics_client,
            )
        else:
            return

        pos = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
        p.createMultiBody(
            baseMass=obj.mass, baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id, basePosition=pos,
            physicsClientId=self._physics_client,
        )
