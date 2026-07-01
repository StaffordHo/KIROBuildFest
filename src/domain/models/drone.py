"""Drone domain model.

Represents multirotor UAVs (quadcopter, hexacopter, etc.)
with 6-DOF rigid body dynamics and rotor thrust models.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import NewType
import math

from .geometry import Vector3, Quaternion, Pose

DroneId = NewType("DroneId", str)


class DroneType(Enum):
    """Supported drone configurations."""
    QUADCOPTER = "quadcopter"      # 4 rotors
    HEXACOPTER = "hexacopter"      # 6 rotors
    OCTOCOPTER = "octocopter"      # 8 rotors
    COAXIAL = "coaxial"            # Coaxial dual rotor


@dataclass(frozen=True)
class RotorConfig:
    """Configuration for a single rotor."""
    position: Vector3              # Relative to drone center of mass
    direction: int = 1             # 1 = CCW, -1 = CW
    max_thrust: float = 5.0        # Maximum thrust (N)
    max_rpm: float = 10000.0       # Maximum RPM
    torque_coefficient: float = 0.01  # Torque per unit thrust


@dataclass(frozen=True)
class DroneConfig:
    """Physical configuration of a drone."""
    drone_type: DroneType = DroneType.QUADCOPTER
    mass: float = 1.5              # kg
    arm_length: float = 0.25       # meters (center to motor)
    inertia_xx: float = 0.01      # kg*m^2
    inertia_yy: float = 0.01
    inertia_zz: float = 0.02
    drag_coefficient: float = 0.1  # Linear drag
    max_tilt_angle: float = math.pi / 4  # Max tilt (45 degrees)
    rotors: list[RotorConfig] = field(default_factory=list)

    def __post_init__(self):
        if not self.rotors:
            # Generate default rotor layout for drone type
            rotors = _generate_rotor_layout(self.drone_type, self.arm_length)
            object.__setattr__(self, "rotors", rotors)


@dataclass
class DroneState:
    """Mutable runtime state of a drone."""
    pose: Pose = field(default_factory=Pose.identity)
    linear_velocity: Vector3 = field(default_factory=Vector3)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    rotor_speeds: list[float] = field(default_factory=list)  # RPM per rotor
    battery_level: float = 100.0   # Percentage
    armed: bool = False

    @property
    def altitude(self) -> float:
        return self.pose.position.z

    @property
    def is_airborne(self) -> bool:
        return self.pose.position.z > 0.05


@dataclass
class DroneCommand:
    """High-level drone control command."""
    thrust: float = 0.0            # Collective thrust (0 to 1)
    roll: float = 0.0              # Roll command (-1 to 1)
    pitch: float = 0.0             # Pitch command (-1 to 1)
    yaw_rate: float = 0.0          # Yaw rate command (-1 to 1)


@dataclass
class Drone:
    """Drone aggregate — represents a complete multirotor UAV.

    Simulates 6-DOF rigid body dynamics with rotor thrust model.
    """
    id: DroneId
    name: str
    config: DroneConfig = field(default_factory=DroneConfig)
    state: DroneState = field(default_factory=DroneState)
    command: DroneCommand = field(default_factory=DroneCommand)

    # PID controller gains for attitude stabilization
    pid_roll_p: float = 5.0
    pid_roll_d: float = 1.0
    pid_pitch_p: float = 5.0
    pid_pitch_d: float = 1.0
    pid_yaw_p: float = 3.0
    pid_yaw_d: float = 0.5

    def arm(self) -> None:
        """Arm the drone (enable motors)."""
        self.state.armed = True
        self.state.rotor_speeds = [0.0] * len(self.config.rotors)

    def disarm(self) -> None:
        """Disarm the drone (disable motors)."""
        self.state.armed = False
        self.state.rotor_speeds = [0.0] * len(self.config.rotors)

    def set_command(self, thrust: float, roll: float, pitch: float, yaw_rate: float) -> None:
        """Set high-level flight command."""
        self.command = DroneCommand(
            thrust=max(0.0, min(1.0, thrust)),
            roll=max(-1.0, min(1.0, roll)),
            pitch=max(-1.0, min(1.0, pitch)),
            yaw_rate=max(-1.0, min(1.0, yaw_rate)),
        )

    def compute_rotor_thrusts(self) -> list[float]:
        """Compute individual rotor thrusts from high-level command.

        Uses a simple mixer for quadcopter layout.
        Returns list of thrust values (N) per rotor.
        """
        if not self.state.armed:
            return [0.0] * len(self.config.rotors)

        base_thrust = self.command.thrust * self.config.mass * 9.81 / len(self.config.rotors)
        roll_component = self.command.roll * 2.0
        pitch_component = self.command.pitch * 2.0
        yaw_component = self.command.yaw_rate * 0.5

        thrusts = []
        num_rotors = len(self.config.rotors)

        if num_rotors == 4:
            # Quadcopter X configuration:
            # Front-Left (0), Front-Right (1), Back-Right (2), Back-Left (3)
            thrusts = [
                base_thrust + pitch_component + roll_component - yaw_component,
                base_thrust + pitch_component - roll_component + yaw_component,
                base_thrust - pitch_component - roll_component - yaw_component,
                base_thrust - pitch_component + roll_component + yaw_component,
            ]
        else:
            # Generic: distribute evenly
            thrusts = [base_thrust] * num_rotors

        # Clamp to max thrust
        max_t = self.config.rotors[0].max_thrust if self.config.rotors else 5.0
        return [max(0.0, min(max_t, t)) for t in thrusts]


def _generate_rotor_layout(drone_type: DroneType, arm_length: float) -> list[RotorConfig]:
    """Generate rotor positions for standard drone configurations."""
    rotors = []

    if drone_type == DroneType.QUADCOPTER:
        # X configuration
        d = arm_length * math.cos(math.pi / 4)
        positions = [
            Vector3(d, d, 0),     # Front-Left
            Vector3(d, -d, 0),    # Front-Right
            Vector3(-d, -d, 0),   # Back-Right
            Vector3(-d, d, 0),    # Back-Left
        ]
        directions = [1, -1, 1, -1]  # Alternating CW/CCW
        for pos, dir in zip(positions, directions):
            rotors.append(RotorConfig(position=pos, direction=dir))

    elif drone_type == DroneType.HEXACOPTER:
        for i in range(6):
            angle = i * math.pi / 3
            pos = Vector3(
                arm_length * math.cos(angle),
                arm_length * math.sin(angle),
                0,
            )
            direction = 1 if i % 2 == 0 else -1
            rotors.append(RotorConfig(position=pos, direction=direction))

    elif drone_type == DroneType.OCTOCOPTER:
        for i in range(8):
            angle = i * math.pi / 4
            pos = Vector3(
                arm_length * math.cos(angle),
                arm_length * math.sin(angle),
                0,
            )
            direction = 1 if i % 2 == 0 else -1
            rotors.append(RotorConfig(position=pos, direction=direction))

    return rotors
