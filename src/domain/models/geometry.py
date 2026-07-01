"""Geometry value objects for spatial representation.

These are immutable value objects used across the domain to represent
positions, orientations, and collision/visual shapes.
"""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Optional
import math


@dataclass(frozen=True)
class Vector3:
    """3D vector representing position or direction."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> Vector3:
        mag = self.magnitude()
        if mag == 0:
            return Vector3(0, 0, 0)
        return Vector3(self.x / mag, self.y / mag, self.z / mag)

    def __add__(self, other: Vector3) -> Vector3:
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vector3) -> Vector3:
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> Vector3:
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def to_list(self) -> list:
        return [self.x, self.y, self.z]


@dataclass(frozen=True)
class Quaternion:
    """Quaternion representing orientation (x, y, z, w)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Create quaternion from Euler angles (radians)."""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        return cls(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

    @classmethod
    def identity(cls) -> Quaternion:
        return cls(0, 0, 0, 1)

    def to_list(self) -> list:
        return [self.x, self.y, self.z, self.w]


@dataclass(frozen=True)
class Pose:
    """6-DOF pose combining position and orientation."""
    position: Vector3 = None
    orientation: Quaternion = None

    def __post_init__(self):
        # Use object.__setattr__ because dataclass is frozen
        if self.position is None:
            object.__setattr__(self, "position", Vector3())
        if self.orientation is None:
            object.__setattr__(self, "orientation", Quaternion.identity())

    @classmethod
    def identity(cls) -> Pose:
        return cls(Vector3(), Quaternion.identity())


class GeometryType(Enum):
    """Supported geometry primitives."""
    BOX = "box"
    CYLINDER = "cylinder"
    SPHERE = "sphere"
    MESH = "mesh"


@dataclass(frozen=True)
class Box:
    """Box geometry defined by half-extents."""
    size_x: float
    size_y: float
    size_z: float
    geometry_type: GeometryType = GeometryType.BOX


@dataclass(frozen=True)
class Cylinder:
    """Cylinder geometry defined by radius and length."""
    radius: float
    length: float
    geometry_type: GeometryType = GeometryType.CYLINDER


@dataclass(frozen=True)
class Sphere:
    """Sphere geometry defined by radius."""
    radius: float
    geometry_type: GeometryType = GeometryType.SPHERE


@dataclass(frozen=True)
class Mesh:
    """Mesh geometry referencing an external file."""
    filename: str
    scale: Vector3 = None
    geometry_type: GeometryType = GeometryType.MESH

    def __post_init__(self):
        if self.scale is None:
            object.__setattr__(self, "scale", Vector3(1, 1, 1))
