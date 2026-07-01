"""Kinematics domain service.

Provides forward and inverse kinematics calculations for robot arms.
"""

from __future__ import annotations
import math
from typing import Optional

from ..models.robot import Robot
from ..models.joint import Joint, JointType
from ..models.geometry import Vector3, Quaternion, Pose


class KinematicsService:
    """Computes forward/inverse kinematics for serial manipulators."""

    def forward_kinematics(self, robot: Robot, joint_positions: dict[str, float]) -> dict[str, Pose]:
        """Compute the pose of each link given joint positions.

        Uses the kinematic chain from base to each link.

        Args:
            robot: The robot model.
            joint_positions: Map of joint_name -> position (rad or m).

        Returns:
            Map of link_name -> world-frame Pose.
        """
        link_poses: dict[str, Pose] = {}

        # Find base link
        base = robot.base_link
        if not base:
            return link_poses

        link_poses[base.name] = robot.base_pose

        # BFS through kinematic tree
        processed = {base.name}
        queue = [base.name]

        while queue:
            parent_name = queue.pop(0)
            parent_pose = link_poses[parent_name]

            for joint in robot.joints.values():
                if joint.parent_link == parent_name and joint.child_link not in processed:
                    # Get joint position
                    q = joint_positions.get(joint.name, joint.state.position)

                    # Compute child pose (simplified DH-like transformation)
                    child_pose = self._compute_child_pose(parent_pose, joint, q)
                    link_poses[joint.child_link] = child_pose

                    processed.add(joint.child_link)
                    queue.append(joint.child_link)

        return link_poses

    def get_end_effector_pose(self, robot: Robot, end_link: str) -> Optional[Pose]:
        """Get the current end-effector pose based on current joint states."""
        positions = {name: j.state.position for name, j in robot.joints.items()}
        all_poses = self.forward_kinematics(robot, positions)
        return all_poses.get(end_link)

    def _compute_child_pose(self, parent_pose: Pose, joint: Joint, q: float) -> Pose:
        """Compute child link pose given parent pose and joint value.

        Simplified transformation:
        T_child = T_parent * T_joint_origin * T_joint_rotation(q)
        """
        # Joint origin offset
        origin = joint.origin
        ox, oy, oz = origin.position.x, origin.position.y, origin.position.z

        # Parent position
        px, py, pz = parent_pose.position.x, parent_pose.position.y, parent_pose.position.z

        if joint.joint_type == JointType.FIXED:
            # No motion, just apply origin offset
            return Pose(
                position=Vector3(px + ox, py + oy, pz + oz),
                orientation=origin.orientation,
            )

        if joint.joint_type in (JointType.REVOLUTE, JointType.CONTINUOUS):
            # Rotation around joint axis
            ax, ay, az = joint.axis.x, joint.axis.y, joint.axis.z

            # Simplified: apply rotation around axis
            # For a proper implementation, use rotation matrices
            cos_q = math.cos(q)
            sin_q = math.sin(q)

            # Position offset rotated by parent orientation (simplified)
            new_x = px + ox * cos_q - oy * sin_q * az
            new_y = py + oy * cos_q + ox * sin_q * az
            new_z = pz + oz

            return Pose(
                position=Vector3(new_x, new_y, new_z),
                orientation=Quaternion.from_euler(
                    q * ax, q * ay, q * az
                ),
            )

        if joint.joint_type == JointType.PRISMATIC:
            # Linear motion along axis
            ax, ay, az = joint.axis.x, joint.axis.y, joint.axis.z
            return Pose(
                position=Vector3(
                    px + ox + q * ax,
                    py + oy + q * ay,
                    pz + oz + q * az,
                ),
                orientation=origin.orientation,
            )

        # Default: just apply origin
        return Pose(
            position=Vector3(px + ox, py + oy, pz + oz),
            orientation=origin.orientation,
        )
