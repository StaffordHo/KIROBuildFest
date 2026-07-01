"""Domain models for RoboSim.

This module defines the core domain entities following DDD principles.
All models are pure Python with no infrastructure dependencies.
"""

from .robot import Robot, RobotId, RobotMetadata
from .joint import Joint, JointId, JointType, JointLimits, JointState
from .link import Link, LinkId, Inertial, Visual, Collision
from .sensor import Sensor, SensorId, SensorType, SensorReading
from .actuator import Actuator, ActuatorId, ControlMode
from .world import World, WorldId, PhysicsConfig, SimulationState, SimStatus
from .geometry import (
    Vector3,
    Quaternion,
    Pose,
    Box,
    Cylinder,
    Sphere,
    Mesh,
    GeometryType,
)

__all__ = [
    "Robot", "RobotId", "RobotMetadata",
    "Joint", "JointId", "JointType", "JointLimits", "JointState",
    "Link", "LinkId", "Inertial", "Visual", "Collision",
    "Sensor", "SensorId", "SensorType", "SensorReading",
    "Actuator", "ActuatorId", "ControlMode",
    "World", "WorldId", "PhysicsConfig", "SimulationState", "SimStatus",
    "Vector3", "Quaternion", "Pose", "Box", "Cylinder", "Sphere", "Mesh", "GeometryType",
]
