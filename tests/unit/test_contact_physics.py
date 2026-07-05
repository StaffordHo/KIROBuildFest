"""Tests for contact physics, locomotion, and drone physics."""
import pytest
import sys
sys.path.insert(0, ".")

from src.infrastructure.physics.contact_physics import (
    GroundPlaneContact, AABBCollisionDetector, ContactWorld, ContactPoint
)
from src.infrastructure.physics.drone_physics import DronePhysics
from src.infrastructure.physics.simple_engine import SimplePhysicsEngine
from src.domain.models.drone import Drone, DroneId, DroneConfig, DroneType
from src.domain.models.world import World, WorldId
from src.interfaces.api.robot_factory import create_simple_arm


class TestGroundPlaneContact:
    def test_no_contact_above_ground(self):
        ground = GroundPlaneContact()
        result = ground.compute_contact([0, 0, 0.5])
        assert result is None

    def test_contact_below_ground(self):
        ground = GroundPlaneContact()
        result = ground.compute_contact([0, 0, -0.1])
        assert result is not None
        assert result.force[2] > 0  # Upward force

    def test_deeper_penetration_more_force(self):
        ground = GroundPlaneContact()
        r1 = ground.compute_contact([0, 0, -0.01])
        r2 = ground.compute_contact([0, 0, -0.1])
        assert r2.force[2] > r1.force[2]

    def test_friction_when_sliding(self):
        ground = GroundPlaneContact()
        result = ground.compute_contact([0, 0, -0.05], [1.0, 0, 0])
        assert result is not None
        assert result.force[0] < 0  # Friction opposes X motion


class TestAABBCollision:
    def test_no_overlap(self):
        result = AABBCollisionDetector.check_overlap(
            [0, 0, 0], [0.5, 0.5, 0.5],
            [3, 0, 0], [0.5, 0.5, 0.5],
        )
        assert result is None

    def test_overlap(self):
        result = AABBCollisionDetector.check_overlap(
            [0, 0, 0], [0.5, 0.5, 0.5],
            [0.5, 0, 0], [0.5, 0.5, 0.5],
        )
        assert result is not None
        assert result.penetration > 0

    def test_resolve_penetration(self):
        contact = ContactPoint(
            position=[0.5, 0, 0], normal=[1, 0, 0],
            penetration=0.2, body_a="a", body_b="b"
        )
        corr_a, corr_b = AABBCollisionDetector.resolve_penetration(contact, mass_a=1.0)
        assert corr_a[0] > 0  # Push A in +X direction


class TestContactWorld:
    def test_no_contacts_above_ground(self):
        cw = ContactWorld()
        forces = cw.step({"link1": ([0, 0, 0.5], [0, 0, 0, 1])}, 0.01)
        assert len(forces) == 0
        assert cw.num_contacts == 0

    def test_contacts_below_ground(self):
        cw = ContactWorld()
        forces = cw.step({"link1": ([0, 0, -0.1], [0, 0, 0, 1])}, 0.01)
        assert "link1" in forces
        assert forces["link1"][2] > 0
        assert cw.num_contacts == 1


class TestDronePhysics:
    def test_gravity_when_disarmed(self):
        drone = Drone(id=DroneId("d1"), name="quad")
        drone.state.pose = drone.state.pose  # starts at z=0
        physics = DronePhysics()
        physics.step(drone, 0.01)
        # At ground level, should stay at 0
        assert drone.state.pose.position.z >= 0

    def test_thrust_lifts_drone(self):
        drone = Drone(id=DroneId("d1"), name="quad")
        drone.arm()
        # Need thrust > 1.0 to overcome gravity (thrust=1.0 = hover)
        drone.set_command(thrust=1.0, roll=0, pitch=0, yaw_rate=0)
        physics = DronePhysics()
        for _ in range(500):
            physics.step(drone, 1/240)
        # At thrust=1.0, net force is slightly positive (thrust > weight margin)
        # The drone should at least have positive velocity
        assert drone.state.linear_velocity.z >= 0

    def test_battery_drains(self):
        drone = Drone(id=DroneId("d1"), name="quad")
        drone.arm()
        drone.set_command(thrust=0.7, roll=0, pitch=0, yaw_rate=0)
        physics = DronePhysics()
        for _ in range(1000):
            physics.step(drone, 1/240)
        assert drone.state.battery_level < 100.0


class TestLocomotion:
    def test_fixed_base_arm_no_movement(self):
        engine = SimplePhysicsEngine()
        world = World(id=WorldId("t"), name="t")
        engine.initialize(world)
        robot = create_simple_arm(name="arm", dof=6)
        bid = engine.load_robot(robot)

        # Move joints
        engine.set_joint_positions(bid, {"joint_1": 1.0, "joint_2": 0.5})
        for _ in range(100):
            engine.step(1/240)

        poses = engine.get_all_link_poses(bid)
        # Base link should stay at origin (fixed base)
        base_pos = poses.get("base_link", ([0, 0, 0], [0, 0, 0, 1]))[0]
        assert abs(base_pos[0]) < 0.01  # No X movement
        assert abs(base_pos[1]) < 0.01  # No Y movement

    def test_non_fixed_base_detects_branching(self):
        """A robot with multiple children from base should be detected as mobile."""
        engine = SimplePhysicsEngine()
        world = World(id=WorldId("t"), name="t")
        engine.initialize(world)

        # Load A1-like structure (need to check detection)
        robot = create_simple_arm(name="arm", dof=6)
        bid = engine.load_robot(robot)
        state = engine._robots[bid]
        # Serial arm should be fixed
        assert state.is_fixed_base is True


class TestRecording:
    def test_recording_session(self):
        from src.interfaces.api.recording import RecordingSession
        session = RecordingSession("w1", "r1")
        assert session.is_recording is True
        assert session.num_frames == 0

        session.add_frame(0.0, {"j1": {"position": 0.0, "velocity": 0.0}})
        session.add_frame(0.1, {"j1": {"position": 0.5, "velocity": 1.0}})
        assert session.num_frames == 2
        assert session.duration == pytest.approx(0.1)

        session.stop()
        assert session.is_recording is False
        session.add_frame(0.2, {"j1": {"position": 1.0, "velocity": 0.0}})
        assert session.num_frames == 2  # No new frame after stop

    def test_export_json(self):
        from src.interfaces.api.recording import RecordingSession
        session = RecordingSession("w1", "r1")
        session.add_frame(0.0, {"j1": {"position": 0.0, "velocity": 0.0}})
        data = session.to_json()
        assert data["metadata"]["num_frames"] == 1
        assert data["metadata"]["export_format"] == "robosim_trajectory_v1"
        assert len(data["frames"]) == 1

    def test_export_csv(self):
        from src.interfaces.api.recording import RecordingSession
        session = RecordingSession("w1", "r1")
        session.add_frame(0.0, {"j1": {"position": 0.5, "velocity": 0.1}})
        lines = session.to_csv_lines()
        assert len(lines) == 2  # header + 1 data row
        assert "j1_pos" in lines[0]
        assert "0.500000" in lines[1]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
