"""Link domain model.

A Link represents a rigid body in the robot's kinematic chain.
Each link has mass properties, visual representation, and collision geometry.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import NewType, Optional, Union

from .geometry import Pose, Vector3, Box, Cylinder, Sphere, Mesh

LinkId = NewType("LinkId", str)

# Union type for all supported geometry shapes
Geometry = Union[Box, Cylinder, Sphere, Mesh]


@dataclass(frozen=True)
class Inertial:
    """Mass and inertia properties of a link."""
    mass: float = 1.0
    origin: Pose = field(default_factory=Pose.identity)
    # Inertia tensor (principal moments)
    ixx: float = 0.001
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.001
    iyz: float = 0.0
    izz: float = 0.001


@dataclass
class Visual:
    """Visual representation of a link (for rendering)."""
    name: str = ""
    origin: Pose = field(default_factory=Pose.identity)
    geometry: Optional[Geometry] = None
    color: Optional[tuple] = None  # RGBA tuple (0-1 range)
    material_name: str = ""


@dataclass
class Collision:
    """Collision geometry for physics interaction."""
    name: str = ""
    origin: Pose = field(default_factory=Pose.identity)
    geometry: Optional[Geometry] = None


@dataclass
class Link:
    """A rigid body element in the robot's kinematic tree.

    Aggregate component of Robot. Links are connected by Joints
    to form the robot's structure.
    """
    id: LinkId
    name: str
    inertial: Inertial = field(default_factory=Inertial)
    visuals: list[Visual] = field(default_factory=list)
    collisions: list[Collision] = field(default_factory=list)

    @property
    def is_base_link(self) -> bool:
        """Whether this is likely the root/base link (heuristic)."""
        return "base" in self.name.lower() or "root" in self.name.lower()
