"""Joint domain model.

A Joint connects two Links in a Robot, defining the degrees of freedom
and constraints of their relative motion.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import NewType, Optional

from .geometry import Vector3, Pose

JointId = NewType("JointId", str)


class JointType(Enum):
    """Supported joint types following URDF specification."""
    REVOLUTE = "revolute"        # Rotation around axis with limits
    CONTINUOUS = "continuous"     # Rotation around axis without limits
    PRISMATIC = "prismatic"      # Linear motion along axis with limits
    FIXED = "fixed"              # No relative motion
    FLOATING = "floating"        # 6-DOF (for mobile bases)
    PLANAR = "planar"            # Motion in a plane


@dataclass(frozen=True)
class JointLimits:
    """Physical limits of a joint's motion."""
    lower: float = 0.0          # Minimum position (rad or m)
    upper: float = 0.0          # Maximum position (rad or m)
    velocity: float = 1.0       # Maximum velocity (rad/s or m/s)
    effort: float = 100.0       # Maximum force/torque (N or Nm)


@dataclass
class JointState:
    """Current state of a joint (mutable, changes during simulation)."""
    position: float = 0.0       # Current position (rad or m)
    velocity: float = 0.0       # Current velocity (rad/s or m/s)
    effort: float = 0.0         # Current applied force/torque


@dataclass
class Joint:
    """A joint connecting two links in a kinematic chain.

    Aggregate component of Robot. Joints define the robot's degrees
    of freedom and enforce physical constraints.
    """
    id: JointId
    name: str
    joint_type: JointType
    parent_link: str            # Name of parent link
    child_link: str             # Name of child link
    origin: Pose = field(default_factory=Pose.identity)
    axis: Vector3 = field(default_factory=lambda: Vector3(0, 0, 1))
    limits: JointLimits = field(default_factory=JointLimits)
    state: JointState = field(default_factory=JointState)
    damping: float = 0.0
    friction: float = 0.0

    @property
    def is_actuated(self) -> bool:
        """Whether this joint can be actively controlled."""
        return self.joint_type not in (JointType.FIXED, JointType.FLOATING)

    @property
    def has_limits(self) -> bool:
        """Whether this joint has position limits."""
        return self.joint_type in (JointType.REVOLUTE, JointType.PRISMATIC)

    def set_position(self, position: float) -> None:
        """Set target position, clamping to limits if applicable."""
        if self.has_limits:
            position = max(self.limits.lower, min(self.limits.upper, position))
        self.state.position = position

    def set_velocity(self, velocity: float) -> None:
        """Set target velocity, clamping to limit."""
        velocity = max(-self.limits.velocity, min(self.limits.velocity, velocity))
        self.state.velocity = velocity
