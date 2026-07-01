"""Drone physics simulation.

6-DOF rigid body dynamics with rotor thrust model.
Simulates gravity, drag, and attitude control.
"""

from __future__ import annotations
import math
from ...domain.models.drone import Drone, DroneState
from ...domain.models.geometry import Vector3, Quaternion, Pose


class DronePhysics:
    """Simulates drone flight dynamics."""

    def step(self, drone: Drone, dt: float) -> None:
        """Advance drone physics by one time step."""
        if not drone.state.armed:
            # Apply gravity only if airborne
            if drone.state.pose.position.z > 0.001:
                self._apply_gravity(drone, dt)
            return

        # Compute rotor thrusts
        thrusts = drone.compute_rotor_thrusts()
        drone.state.rotor_speeds = thrusts

        # Total forces and torques
        total_thrust = sum(thrusts)
        mass = drone.config.mass
        g = 9.81

        # Current orientation as euler angles (simplified)
        state = drone.state
        pos = state.pose.position
        vel = state.linear_velocity
        ang_vel = state.angular_velocity

        # Attitude from commands (simplified stabilized model)
        target_roll = drone.command.roll * drone.config.max_tilt_angle
        target_pitch = drone.command.pitch * drone.config.max_tilt_angle
        target_yaw_rate = drone.command.yaw_rate * 3.0  # rad/s

        # Get current attitude (simplified — track as euler)
        current_roll = getattr(state, '_roll', 0.0)
        current_pitch = getattr(state, '_pitch', 0.0)
        current_yaw = getattr(state, '_yaw', 0.0)

        # PD attitude control
        roll_torque = drone.pid_roll_p * (target_roll - current_roll) - drone.pid_roll_d * ang_vel.x
        pitch_torque = drone.pid_pitch_p * (target_pitch - current_pitch) - drone.pid_pitch_d * ang_vel.y
        yaw_torque = drone.pid_yaw_p * (target_yaw_rate - ang_vel.z)

        # Angular acceleration
        alpha_x = roll_torque / drone.config.inertia_xx
        alpha_y = pitch_torque / drone.config.inertia_yy
        alpha_z = yaw_torque / drone.config.inertia_zz

        # Update angular velocity
        new_ang_vel = Vector3(
            ang_vel.x + alpha_x * dt,
            ang_vel.y + alpha_y * dt,
            ang_vel.z + alpha_z * dt,
        )

        # Update attitude
        new_roll = current_roll + new_ang_vel.x * dt
        new_pitch = current_pitch + new_ang_vel.y * dt
        new_yaw = current_yaw + new_ang_vel.z * dt

        # Linear forces
        # Thrust in body frame -> world frame
        thrust_world_x = total_thrust * (-math.sin(new_pitch))
        thrust_world_y = total_thrust * (math.sin(new_roll) * math.cos(new_pitch))
        thrust_world_z = total_thrust * (math.cos(new_roll) * math.cos(new_pitch))

        # Drag force
        drag = drone.config.drag_coefficient
        drag_x = -drag * vel.x
        drag_y = -drag * vel.y
        drag_z = -drag * vel.z

        # Net acceleration
        ax = (thrust_world_x + drag_x) / mass
        ay = (thrust_world_y + drag_y) / mass
        az = (thrust_world_z + drag_z) / mass - g

        # Update velocity
        new_vel = Vector3(
            vel.x + ax * dt,
            vel.y + ay * dt,
            vel.z + az * dt,
        )

        # Update position
        new_pos = Vector3(
            pos.x + new_vel.x * dt,
            pos.y + new_vel.y * dt,
            max(0.0, pos.z + new_vel.z * dt),  # Ground constraint
        )

        # Ground collision
        if new_pos.z <= 0.0:
            new_pos = Vector3(new_pos.x, new_pos.y, 0.0)
            new_vel = Vector3(new_vel.x * 0.5, new_vel.y * 0.5, 0.0)
            new_ang_vel = Vector3(new_ang_vel.x * 0.5, new_ang_vel.y * 0.5, new_ang_vel.z * 0.5)
            new_roll *= 0.9
            new_pitch *= 0.9

        # Store state
        state.pose = Pose(
            position=new_pos,
            orientation=Quaternion.from_euler(new_roll, new_pitch, new_yaw),
        )
        state.linear_velocity = new_vel
        state.angular_velocity = new_ang_vel

        # Store euler angles for next step
        state._roll = new_roll
        state._pitch = new_pitch
        state._yaw = new_yaw

        # Battery drain (simplified)
        power_draw = sum(thrusts) / (drone.config.rotors[0].max_thrust * len(drone.config.rotors))
        state.battery_level = max(0.0, state.battery_level - power_draw * 0.01 * dt)

    def _apply_gravity(self, drone: Drone, dt: float) -> None:
        """Apply gravity when not armed (freefall or on ground)."""
        state = drone.state
        pos = state.pose.position
        vel = state.linear_velocity

        new_vel = Vector3(vel.x, vel.y, vel.z - 9.81 * dt)
        new_z = max(0.0, pos.z + new_vel.z * dt)

        if new_z <= 0.0:
            new_vel = Vector3(0, 0, 0)

        state.pose = Pose(
            position=Vector3(pos.x, pos.y, new_z),
            orientation=state.pose.orientation,
        )
        state.linear_velocity = new_vel
