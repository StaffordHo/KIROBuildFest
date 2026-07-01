"""Actuator domain model.

Actuators apply forces/torques to joints, translating control
commands into physical action in the simulation.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import NewType

ActuatorId = NewType("ActuatorId", str)


class ControlMode(Enum):
    """Supported actuator control modes."""
    POSITION = "position"         # PID position control
    VELOCITY = "velocity"         # Velocity control
    EFFORT = "effort"             # Direct force/torque control
    IMPEDANCE = "impedance"       # Impedance/compliance control


@dataclass
class Actuator:
    """An actuator driving a robot joint.

    Actuators bridge the gap between control commands and
    physical forces applied in the simulation.
    """
    id: ActuatorId
    name: str
    joint_name: str               # Joint this actuator drives
    control_mode: ControlMode = ControlMode.POSITION

    # PID gains (for position/velocity control)
    kp: float = 100.0             # Proportional gain
    ki: float = 0.0               # Integral gain
    kd: float = 10.0              # Derivative gain

    # Physical limits
    max_effort: float = 100.0     # Maximum force/torque (N or Nm)
    max_velocity: float = 2.0     # Maximum velocity (rad/s or m/s)

    # Current command
    target: float = 0.0           # Target value (interpretation depends on mode)

    def set_target(self, value: float) -> None:
        """Set the actuator target, respecting physical limits."""
        if self.control_mode == ControlMode.VELOCITY:
            value = max(-self.max_velocity, min(self.max_velocity, value))
        elif self.control_mode == ControlMode.EFFORT:
            value = max(-self.max_effort, min(self.max_effort, value))
        self.target = value

    def compute_effort(self, current_position: float, current_velocity: float, dt: float) -> float:
        """Compute effort output based on control mode and PID.

        Args:
            current_position: Current joint position.
            current_velocity: Current joint velocity.
            dt: Time step.

        Returns:
            Effort (force/torque) to apply.
        """
        if self.control_mode == ControlMode.EFFORT:
            return max(-self.max_effort, min(self.max_effort, self.target))

        if self.control_mode == ControlMode.POSITION:
            error = self.target - current_position
            effort = self.kp * error - self.kd * current_velocity
            return max(-self.max_effort, min(self.max_effort, effort))

        if self.control_mode == ControlMode.VELOCITY:
            error = self.target - current_velocity
            effort = self.kp * error
            return max(-self.max_effort, min(self.max_effort, effort))

        return 0.0
