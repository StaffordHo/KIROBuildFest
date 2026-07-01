"""URDF file loader — converts URDF XML into domain models.

Supports importing standard URDF robot descriptions (the de-facto
standard for robot model exchange in ROS ecosystem).
"""

from __future__ import annotations
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional
import uuid

from ...domain.models.robot import Robot, RobotId, RobotMetadata
from ...domain.models.joint import Joint, JointId, JointType, JointLimits
from ...domain.models.link import Link, LinkId, Inertial, Visual, Collision
from ...domain.models.geometry import (
    Vector3, Quaternion, Pose, Box, Cylinder, Sphere, Mesh,
)


# Map URDF joint type strings to domain enum
_JOINT_TYPE_MAP = {
    "revolute": JointType.REVOLUTE,
    "continuous": JointType.CONTINUOUS,
    "prismatic": JointType.PRISMATIC,
    "fixed": JointType.FIXED,
    "floating": JointType.FLOATING,
    "planar": JointType.PLANAR,
}


class URDFLoader:
    """Parses URDF files and creates domain Robot models."""

    def load_from_file(self, filepath: str) -> Robot:
        """Load a robot from a URDF file.

        Args:
            filepath: Path to the .urdf file.

        Returns:
            Fully populated Robot domain model.
        """
        tree = ET.parse(filepath)
        root = tree.getroot()
        return self._parse_robot(root, source_url=filepath)

    def load_from_string(self, urdf_string: str, name: str = "imported") -> Robot:
        """Load a robot from a URDF XML string.

        Args:
            urdf_string: URDF XML content.
            name: Fallback name if not specified in URDF.

        Returns:
            Fully populated Robot domain model.
        """
        root = ET.fromstring(urdf_string)
        return self._parse_robot(root, source_url="")

    def _parse_robot(self, root: ET.Element, source_url: str) -> Robot:
        """Parse the root <robot> element."""
        robot_name = root.attrib.get("name", "unnamed_robot")

        robot = Robot(
            id=RobotId(str(uuid.uuid4())),
            metadata=RobotMetadata(
                name=robot_name,
                description=f"Imported from URDF: {robot_name}",
                source_format="urdf",
                source_url=source_url,
            ),
        )

        # Parse links first (joints reference them)
        for link_elem in root.findall("link"):
            link = self._parse_link(link_elem)
            robot.add_link(link)

        # Parse joints
        for joint_elem in root.findall("joint"):
            joint = self._parse_joint(joint_elem)
            # Only add if parent/child links exist
            if joint.parent_link in robot.links and joint.child_link in robot.links:
                robot.joints[joint.name] = joint

        return robot

    def _parse_link(self, elem: ET.Element) -> Link:
        """Parse a <link> element."""
        name = elem.attrib.get("name", "unnamed_link")

        # Parse inertial
        inertial = Inertial()
        inertial_elem = elem.find("inertial")
        if inertial_elem is not None:
            inertial = self._parse_inertial(inertial_elem)

        # Parse visuals
        visuals = []
        for vis_elem in elem.findall("visual"):
            visual = self._parse_visual(vis_elem)
            if visual:
                visuals.append(visual)

        # Parse collisions
        collisions = []
        for col_elem in elem.findall("collision"):
            collision = self._parse_collision(col_elem)
            if collision:
                collisions.append(collision)

        return Link(
            id=LinkId(name),
            name=name,
            inertial=inertial,
            visuals=visuals,
            collisions=collisions,
        )

    def _parse_joint(self, elem: ET.Element) -> Joint:
        """Parse a <joint> element."""
        name = elem.attrib.get("name", "unnamed_joint")
        jtype_str = elem.attrib.get("type", "fixed")
        joint_type = _JOINT_TYPE_MAP.get(jtype_str, JointType.FIXED)

        parent_elem = elem.find("parent")
        child_elem = elem.find("child")
        parent_link = parent_elem.attrib.get("link", "") if parent_elem is not None else ""
        child_link = child_elem.attrib.get("link", "") if child_elem is not None else ""

        # Parse origin
        origin = self._parse_origin(elem.find("origin"))

        # Parse axis
        axis = Vector3(0, 0, 1)
        axis_elem = elem.find("axis")
        if axis_elem is not None:
            axis = self._parse_xyz(axis_elem.attrib.get("xyz", "0 0 1"))

        # Parse limits
        limits = JointLimits()
        limit_elem = elem.find("limit")
        if limit_elem is not None:
            limits = JointLimits(
                lower=float(limit_elem.attrib.get("lower", "0")),
                upper=float(limit_elem.attrib.get("upper", "0")),
                velocity=float(limit_elem.attrib.get("velocity", "1")),
                effort=float(limit_elem.attrib.get("effort", "100")),
            )

        # Parse dynamics
        damping = 0.0
        friction = 0.0
        dynamics_elem = elem.find("dynamics")
        if dynamics_elem is not None:
            damping = float(dynamics_elem.attrib.get("damping", "0"))
            friction = float(dynamics_elem.attrib.get("friction", "0"))

        return Joint(
            id=JointId(name),
            name=name,
            joint_type=joint_type,
            parent_link=parent_link,
            child_link=child_link,
            origin=origin,
            axis=axis,
            limits=limits,
            damping=damping,
            friction=friction,
        )

    def _parse_inertial(self, elem: ET.Element) -> Inertial:
        """Parse an <inertial> element."""
        mass = 1.0
        mass_elem = elem.find("mass")
        if mass_elem is not None:
            mass = float(mass_elem.attrib.get("value", "1"))

        origin = self._parse_origin(elem.find("origin"))

        ixx = iyy = izz = 0.001
        ixy = ixz = iyz = 0.0
        inertia_elem = elem.find("inertia")
        if inertia_elem is not None:
            ixx = float(inertia_elem.attrib.get("ixx", "0.001"))
            ixy = float(inertia_elem.attrib.get("ixy", "0"))
            ixz = float(inertia_elem.attrib.get("ixz", "0"))
            iyy = float(inertia_elem.attrib.get("iyy", "0.001"))
            iyz = float(inertia_elem.attrib.get("iyz", "0"))
            izz = float(inertia_elem.attrib.get("izz", "0.001"))

        return Inertial(
            mass=mass, origin=origin,
            ixx=ixx, ixy=ixy, ixz=ixz,
            iyy=iyy, iyz=iyz, izz=izz,
        )

    def _parse_visual(self, elem: ET.Element) -> Optional[Visual]:
        """Parse a <visual> element."""
        name = elem.attrib.get("name", "")
        origin = self._parse_origin(elem.find("origin"))
        geometry = self._parse_geometry(elem.find("geometry"))

        color = None
        material_elem = elem.find("material")
        if material_elem is not None:
            color_elem = material_elem.find("color")
            if color_elem is not None:
                rgba_str = color_elem.attrib.get("rgba", "0.8 0.8 0.8 1.0")
                color = tuple(float(v) for v in rgba_str.split())

        return Visual(name=name, origin=origin, geometry=geometry, color=color)

    def _parse_collision(self, elem: ET.Element) -> Optional[Collision]:
        """Parse a <collision> element."""
        name = elem.attrib.get("name", "")
        origin = self._parse_origin(elem.find("origin"))
        geometry = self._parse_geometry(elem.find("geometry"))
        return Collision(name=name, origin=origin, geometry=geometry)

    def _parse_geometry(self, elem: Optional[ET.Element]):
        """Parse a <geometry> element."""
        if elem is None:
            return None

        box_elem = elem.find("box")
        if box_elem is not None:
            size = box_elem.attrib.get("size", "0.1 0.1 0.1").split()
            return Box(float(size[0]), float(size[1]), float(size[2]))

        cyl_elem = elem.find("cylinder")
        if cyl_elem is not None:
            return Cylinder(
                radius=float(cyl_elem.attrib.get("radius", "0.05")),
                length=float(cyl_elem.attrib.get("length", "0.1")),
            )

        sphere_elem = elem.find("sphere")
        if sphere_elem is not None:
            return Sphere(radius=float(sphere_elem.attrib.get("radius", "0.05")))

        mesh_elem = elem.find("mesh")
        if mesh_elem is not None:
            filename = mesh_elem.attrib.get("filename", "")
            scale_str = mesh_elem.attrib.get("scale", "1 1 1")
            scale_vals = [float(v) for v in scale_str.split()]
            return Mesh(filename=filename, scale=Vector3(*scale_vals))

        return None

    def _parse_origin(self, elem: Optional[ET.Element]) -> Pose:
        """Parse an <origin> element into a Pose."""
        if elem is None:
            return Pose.identity()

        xyz_str = elem.attrib.get("xyz", "0 0 0")
        rpy_str = elem.attrib.get("rpy", "0 0 0")

        position = self._parse_xyz(xyz_str)
        rpy = [float(v) for v in rpy_str.split()]
        orientation = Quaternion.from_euler(rpy[0], rpy[1], rpy[2])

        return Pose(position=position, orientation=orientation)

    def _parse_xyz(self, xyz_str: str) -> Vector3:
        """Parse 'x y z' string into Vector3."""
        vals = [float(v) for v in xyz_str.split()]
        return Vector3(vals[0], vals[1], vals[2]) if len(vals) == 3 else Vector3()
