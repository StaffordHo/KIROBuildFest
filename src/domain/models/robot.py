"""Robot aggregate root.

The Robot is the primary aggregate in the domain. It owns and manages
its Links, Joints, Sensors, and Actuators as a consistent unit.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import NewType, Optional

from .joint import Joint, JointId, JointState
from .link import Link, LinkId
from .sensor import Sensor, SensorId
from .actuator import Actuator, ActuatorId
from .geometry import Pose

RobotId = NewType("RobotId", str)


@dataclass(frozen=True)
class RobotMetadata:
    """Descriptive metadata for a robot model."""
    name: str
    description: str = ""
    author: str = ""
    version: str = "1.0.0"
    source_format: str = "urdf"   # urdf, sdf, custom
    source_url: str = ""


@dataclass
class Robot:
    """Aggregate root representing a complete robot system.

    A Robot is a kinematic tree of Links connected by Joints,
    with Sensors for feedback and Actuators for control.
    Enforces invariants across its components.
    """
    id: RobotId
    metadata: RobotMetadata
    links: dict[str, Link] = field(default_factory=dict)
    joints: dict[str, Joint] = field(default_factory=dict)
    sensors: dict[str, Sensor] = field(default_factory=dict)
    actuators: dict[str, Actuator] = field(default_factory=dict)
    base_pose: Pose = field(default_factory=Pose.identity)

    # --- Aggregate operations ---

    def add_link(self, link: Link) -> None:
        """Add a link to the robot."""
        if link.name in self.links:
            raise ValueError(f"Link '{link.name}' already exists in robot '{self.metadata.name}'")
        self.links[link.name] = link

    def add_joint(self, joint: Joint) -> None:
        """Add a joint, validating that parent/child links exist."""
        if joint.parent_link not in self.links:
            raise ValueError(
                f"Parent link '{joint.parent_link}' not found. "
                f"Available links: {list(self.links.keys())}"
            )
        if joint.child_link not in self.links:
            raise ValueError(
                f"Child link '{joint.child_link}' not found. "
                f"Available links: {list(self.links.keys())}"
            )
        if joint.name in self.joints:
            raise ValueError(f"Joint '{joint.name}' already exists")
        self.joints[joint.name] = joint

    def add_sensor(self, sensor: Sensor) -> None:
        """Add a sensor, validating parent link exists."""
        if sensor.parent_link not in self.links:
            raise ValueError(f"Parent link '{sensor.parent_link}' not found for sensor '{sensor.name}'")
        self.sensors[sensor.name] = sensor

    def add_actuator(self, actuator: Actuator) -> None:
        """Add an actuator, validating the joint exists."""
        if actuator.joint_name not in self.joints:
            raise ValueError(f"Joint '{actuator.joint_name}' not found for actuator '{actuator.name}'")
        self.actuators[actuator.name] = actuator

    # --- Queries ---

    @property
    def dof(self) -> int:
        """Degrees of freedom (number of actuated joints)."""
        return sum(1 for j in self.joints.values() if j.is_actuated)

    @property
    def base_link(self) -> Optional[Link]:
        """Find the root/base link of the kinematic tree."""
        child_links = {j.child_link for j in self.joints.values()}
        for link_name, link in self.links.items():
            if link_name not in child_links:
                return link
        return next(iter(self.links.values()), None)

    def get_joint_states(self) -> dict[str, JointState]:
        """Get current state of all joints."""
        return {name: joint.state for name, joint in self.joints.items()}

    def set_joint_positions(self, positions: dict[str, float]) -> None:
        """Set multiple joint positions at once."""
        for joint_name, position in positions.items():
            if joint_name in self.joints:
                self.joints[joint_name].set_position(position)

    def get_kinematic_chain(self, end_link: str) -> list[Joint]:
        """Get the chain of joints from base to end effector."""
        chain = []
        current = end_link
        visited = set()

        while current and current not in visited:
            visited.add(current)
            for joint in self.joints.values():
                if joint.child_link == current:
                    chain.append(joint)
                    current = joint.parent_link
                    break
            else:
                break

        chain.reverse()
        return chain
