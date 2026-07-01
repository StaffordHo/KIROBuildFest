"""Sensor domain model.

Sensors provide feedback from the simulated environment,
mimicking real hardware sensor outputs.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import NewType, Optional, Any
from datetime import datetime

from .geometry import Vector3, Quaternion

SensorId = NewType("SensorId", str)


class SensorType(Enum):
    """Supported sensor types."""
    FORCE_TORQUE = "force_torque"    # 6-axis force/torque
    IMU = "imu"                       # Inertial measurement unit
    CAMERA = "camera"                 # RGB/Depth camera
    LIDAR = "lidar"                   # Laser range finder
    JOINT_ENCODER = "joint_encoder"   # Position/velocity feedback
    CONTACT = "contact"               # Binary contact sensor
    PROXIMITY = "proximity"           # Distance to nearest object


@dataclass
class SensorReading:
    """A timestamped sensor measurement."""
    sensor_id: SensorId
    timestamp: float  # Simulation time in seconds
    data: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def force_torque(
        cls, sensor_id: SensorId, timestamp: float,
        force: Vector3, torque: Vector3
    ) -> SensorReading:
        return cls(
            sensor_id=sensor_id,
            timestamp=timestamp,
            data={
                "force": {"x": force.x, "y": force.y, "z": force.z},
                "torque": {"x": torque.x, "y": torque.y, "z": torque.z},
            },
        )

    @classmethod
    def imu(
        cls, sensor_id: SensorId, timestamp: float,
        linear_acceleration: Vector3,
        angular_velocity: Vector3,
        orientation: Quaternion,
    ) -> SensorReading:
        return cls(
            sensor_id=sensor_id,
            timestamp=timestamp,
            data={
                "linear_acceleration": {
                    "x": linear_acceleration.x,
                    "y": linear_acceleration.y,
                    "z": linear_acceleration.z,
                },
                "angular_velocity": {
                    "x": angular_velocity.x,
                    "y": angular_velocity.y,
                    "z": angular_velocity.z,
                },
                "orientation": {
                    "x": orientation.x, "y": orientation.y,
                    "z": orientation.z, "w": orientation.w,
                },
            },
        )


@dataclass
class Sensor:
    """A sensor attached to a robot link.

    Sensors are domain entities that define what measurements
    are available. The infrastructure layer handles actual
    data generation from the physics engine.
    """
    id: SensorId
    name: str
    sensor_type: SensorType
    parent_link: str              # Link this sensor is attached to
    update_rate: float = 100.0    # Hz
    noise_stddev: float = 0.0     # Gaussian noise standard deviation
    enabled: bool = True

    # Type-specific configuration
    config: dict[str, Any] = field(default_factory=dict)

    @property
    def update_period(self) -> float:
        """Time between updates in seconds."""
        return 1.0 / self.update_rate if self.update_rate > 0 else float("inf")
