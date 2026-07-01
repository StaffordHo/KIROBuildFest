"""Mobile robot domain model.

Represents wheeled/tracked ground robots with differential drive,
Ackermann steering, or omnidirectional movement.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import NewType
import math

from .geometry import Vector3, Quaternion, Pose

MobileRobotId = NewType("MobileRobotId", str)


class DriveType(Enum):
    """Supported drive configurations."""
    DIFFERENTIAL = "differential"    # Two-wheel differential drive (e.g., TurtleBot)
    ACKERMANN = "ackermann"          # Car-like steering (e.g., RC car)
    OMNI = "omnidirectional"         # Mecanum/omni wheels (e.g., Kuka youBot base)
    SKID_STEER = "skid_steer"       # Tank-like tracks


@dataclass(frozen=True)
class MobileRobotConfig:
    """Physical configuration of a mobile robot."""
    drive_type: DriveType = DriveType.DIFFERENTIAL
    mass: float = 10.0             # kg
    wheel_radius: float = 0.05    # meters
    wheel_base: float = 0.3       # Distance between wheels (m)
    track_width: float = 0.3      # Width between left/right wheels (m)
    max_linear_speed: float = 1.0  # m/s
    max_angular_speed: float = 2.0  # rad/s
    max_acceleration: float = 2.0  # m/s^2
    body_length: float = 0.4      # meters
    body_width: float = 0.3       # meters
    body_height: float = 0.15     # meters


@dataclass
class MobileRobotState:
    """Mutable runtime state of a mobile robot."""
    pose: Pose = field(default_factory=lambda: Pose(
        position=Vector3(0, 0, 0),
        orientation=Quaternion.identity(),
    ))
    linear_velocity: float = 0.0   # m/s (forward)
    angular_velocity: float = 0.0  # rad/s (yaw rate)
    heading: float = 0.0           # radians (yaw angle)
    left_wheel_speed: float = 0.0  # rad/s
    right_wheel_speed: float = 0.0  # rad/s
    odometry_x: float = 0.0       # Cumulative x displacement
    odometry_y: float = 0.0       # Cumulative y displacement
    odometry_theta: float = 0.0   # Cumulative heading


@dataclass
class MobileRobotCommand:
    """Velocity command for mobile robot."""
    linear_x: float = 0.0         # Forward speed (m/s)
    linear_y: float = 0.0         # Lateral speed (m/s) - omni only
    angular_z: float = 0.0        # Yaw rate (rad/s)


@dataclass
class MobileRobot:
    """Mobile robot aggregate — ground-based wheeled/tracked robot.

    Simulates 2D planar motion with different drive kinematics.
    """
    id: MobileRobotId
    name: str
    config: MobileRobotConfig = field(default_factory=MobileRobotConfig)
    state: MobileRobotState = field(default_factory=MobileRobotState)
    command: MobileRobotCommand = field(default_factory=MobileRobotCommand)

    def set_velocity(self, linear_x: float, angular_z: float, linear_y: float = 0.0) -> None:
        """Set velocity command."""
        self.command = MobileRobotCommand(
            linear_x=max(-self.config.max_linear_speed, min(self.config.max_linear_speed, linear_x)),
            linear_y=max(-self.config.max_linear_speed, min(self.config.max_linear_speed, linear_y)),
            angular_z=max(-self.config.max_angular_speed, min(self.config.max_angular_speed, angular_z)),
        )

    def step(self, dt: float) -> None:
        """Update state based on current command and kinematics model."""
        if self.config.drive_type == DriveType.DIFFERENTIAL:
            self._step_differential(dt)
        elif self.config.drive_type == DriveType.OMNI:
            self._step_omnidirectional(dt)
        elif self.config.drive_type == DriveType.ACKERMANN:
            self._step_ackermann(dt)
        else:
            self._step_differential(dt)

    def _step_differential(self, dt: float) -> None:
        """Differential drive kinematics update."""
        v = self.command.linear_x
        omega = self.command.angular_z

        # Simple acceleration model
        accel = self.config.max_acceleration
        dv = v - self.state.linear_velocity
        if abs(dv) > accel * dt:
            dv = math.copysign(accel * dt, dv)
        self.state.linear_velocity += dv
        self.state.angular_velocity = omega

        # Update pose
        theta = self.state.heading
        dx = self.state.linear_velocity * math.cos(theta) * dt
        dy = self.state.linear_velocity * math.sin(theta) * dt
        dtheta = self.state.angular_velocity * dt

        new_x = self.state.pose.position.x + dx
        new_y = self.state.pose.position.y + dy
        self.state.heading += dtheta

        self.state.pose = Pose(
            position=Vector3(new_x, new_y, 0),
            orientation=Quaternion.from_euler(0, 0, self.state.heading),
        )

        # Update wheel speeds
        r = self.config.wheel_radius
        L = self.config.track_width
        self.state.left_wheel_speed = (self.state.linear_velocity - omega * L / 2) / r
        self.state.right_wheel_speed = (self.state.linear_velocity + omega * L / 2) / r

        # Odometry
        self.state.odometry_x += dx
        self.state.odometry_y += dy
        self.state.odometry_theta = self.state.heading

    def _step_omnidirectional(self, dt: float) -> None:
        """Omnidirectional drive kinematics (mecanum wheels)."""
        vx = self.command.linear_x
        vy = self.command.linear_y
        omega = self.command.angular_z

        theta = self.state.heading

        # World-frame velocity
        dx = (vx * math.cos(theta) - vy * math.sin(theta)) * dt
        dy = (vx * math.sin(theta) + vy * math.cos(theta)) * dt
        dtheta = omega * dt

        new_x = self.state.pose.position.x + dx
        new_y = self.state.pose.position.y + dy
        self.state.heading += dtheta

        self.state.pose = Pose(
            position=Vector3(new_x, new_y, 0),
            orientation=Quaternion.from_euler(0, 0, self.state.heading),
        )
        self.state.linear_velocity = math.sqrt(vx**2 + vy**2)
        self.state.angular_velocity = omega

    def _step_ackermann(self, dt: float) -> None:
        """Ackermann (car-like) steering kinematics."""
        v = self.command.linear_x
        # Angular command interpreted as steering angle
        steering_angle = self.command.angular_z * 0.5  # Scale to max ~30 degrees

        accel = self.config.max_acceleration
        dv = v - self.state.linear_velocity
        if abs(dv) > accel * dt:
            dv = math.copysign(accel * dt, dv)
        self.state.linear_velocity += dv

        theta = self.state.heading
        L = self.config.wheel_base

        if abs(steering_angle) > 0.001:
            turning_radius = L / math.tan(steering_angle)
            omega = self.state.linear_velocity / turning_radius
        else:
            omega = 0.0

        dx = self.state.linear_velocity * math.cos(theta) * dt
        dy = self.state.linear_velocity * math.sin(theta) * dt
        dtheta = omega * dt

        new_x = self.state.pose.position.x + dx
        new_y = self.state.pose.position.y + dy
        self.state.heading += dtheta

        self.state.pose = Pose(
            position=Vector3(new_x, new_y, 0),
            orientation=Quaternion.from_euler(0, 0, self.state.heading),
        )
        self.state.angular_velocity = omega

    def get_state_dict(self) -> dict:
        """Serialize state for API/WebSocket streaming."""
        return {
            "position": [
                self.state.pose.position.x,
                self.state.pose.position.y,
                self.state.pose.position.z,
            ],
            "heading": self.state.heading,
            "linear_velocity": self.state.linear_velocity,
            "angular_velocity": self.state.angular_velocity,
            "odometry": {
                "x": self.state.odometry_x,
                "y": self.state.odometry_y,
                "theta": self.state.odometry_theta,
            },
        }
