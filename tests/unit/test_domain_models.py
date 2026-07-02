"""Unit tests for domain models — Robot, Joint, Link, World, Drone, MobileRobot."""
import pytest
import sys
sys.path.insert(0, ".")

from src.domain.models.geometry import Vector3, Quaternion, Pose
from src.domain.models.joint import Joint, JointId, JointType, JointLimits, JointState
from src.domain.models.link import Link, LinkId, Inertial
from src.domain.models.robot import Robot, RobotId, RobotMetadata
from src.domain.models.world import World, WorldId, PhysicsConfig, SimStatus
from src.domain.models.actuator import Actuator, ActuatorId, ControlMode
from src.domain.models.drone import Drone, DroneId, DroneConfig, DroneType
from src.domain.models.mobile_robot import MobileRobot, MobileRobotId, MobileRobotConfig, DriveType


class TestVector3:
    def test_magnitude(self):
        v = Vector3(3, 4, 0)
        assert abs(v.magnitude() - 5.0) < 1e-10

    def test_normalized(self):
        v = Vector3(0, 0, 5)
        n = v.normalized()
        assert abs(n.z - 1.0) < 1e-10
        assert abs(n.magnitude() - 1.0) < 1e-10

    def test_addition(self):
        a = Vector3(1, 2, 3)
        b = Vector3(4, 5, 6)
        c = a + b
        assert c.x == 5 and c.y == 7 and c.z == 9

    def test_scalar_multiply(self):
        v = Vector3(1, 2, 3) * 2
        assert v.x == 2 and v.y == 4 and v.z == 6


class TestQuaternion:
    def test_identity(self):
        q = Quaternion.identity()
        assert q.w == 1.0 and q.x == 0 and q.y == 0 and q.z == 0

    def test_from_euler_zero(self):
        q = Quaternion.from_euler(0, 0, 0)
        assert abs(q.w - 1.0) < 1e-10


class TestJoint:
    def test_revolute_is_actuated(self):
        j = Joint(id=JointId("j1"), name="j1", joint_type=JointType.REVOLUTE,
                  parent_link="a", child_link="b")
        assert j.is_actuated is True

    def test_fixed_not_actuated(self):
        j = Joint(id=JointId("j1"), name="j1", joint_type=JointType.FIXED,
                  parent_link="a", child_link="b")
        assert j.is_actuated is False

    def test_set_position_clamps_to_limits(self):
        j = Joint(id=JointId("j1"), name="j1", joint_type=JointType.REVOLUTE,
                  parent_link="a", child_link="b",
                  limits=JointLimits(lower=-1.0, upper=1.0))
        j.set_position(5.0)
        assert j.state.position == 1.0
        j.set_position(-5.0)
        assert j.state.position == -1.0

    def test_continuous_no_clamp(self):
        j = Joint(id=JointId("j1"), name="j1", joint_type=JointType.CONTINUOUS,
                  parent_link="a", child_link="b")
        j.set_position(10.0)
        assert j.state.position == 10.0


class TestRobot:
    def _make_robot(self):
        r = Robot(id=RobotId("r1"), metadata=RobotMetadata(name="test"))
        r.add_link(Link(id=LinkId("base"), name="base"))
        r.add_link(Link(id=LinkId("link1"), name="link1"))
        r.add_joint(Joint(id=JointId("j1"), name="j1",
                          joint_type=JointType.REVOLUTE,
                          parent_link="base", child_link="link1"))
        return r

    def test_add_link(self):
        r = Robot(id=RobotId("r1"), metadata=RobotMetadata(name="test"))
        r.add_link(Link(id=LinkId("base"), name="base"))
        assert "base" in r.links

    def test_add_duplicate_link_raises(self):
        r = Robot(id=RobotId("r1"), metadata=RobotMetadata(name="test"))
        r.add_link(Link(id=LinkId("base"), name="base"))
        with pytest.raises(ValueError):
            r.add_link(Link(id=LinkId("base"), name="base"))

    def test_add_joint_validates_links(self):
        r = Robot(id=RobotId("r1"), metadata=RobotMetadata(name="test"))
        r.add_link(Link(id=LinkId("base"), name="base"))
        with pytest.raises(ValueError):
            r.add_joint(Joint(id=JointId("j1"), name="j1",
                              joint_type=JointType.REVOLUTE,
                              parent_link="base", child_link="missing"))

    def test_dof(self):
        r = self._make_robot()
        assert r.dof == 1

    def test_base_link(self):
        r = self._make_robot()
        assert r.base_link.name == "base"

    def test_set_joint_positions(self):
        r = self._make_robot()
        # Joint has limits (0, 0) by default — need to set wider limits
        r.joints["j1"].limits = JointLimits(lower=-3.14, upper=3.14)
        r.set_joint_positions({"j1": 0.5})
        assert r.joints["j1"].state.position == 0.5


class TestWorld:
    def test_create_world(self):
        w = World(id=WorldId("w1"), name="test")
        assert w.state.status == SimStatus.IDLE

    def test_start_stop(self):
        w = World(id=WorldId("w1"), name="test")
        w.start()
        assert w.is_running
        w.pause()
        assert w.state.status == SimStatus.PAUSED
        w.stop()
        assert w.state.status == SimStatus.STOPPED

    def test_add_robot(self):
        w = World(id=WorldId("w1"), name="test")
        r = Robot(id=RobotId("r1"), metadata=RobotMetadata(name="arm"))
        w.add_robot(r)
        assert "r1" in w.robots


class TestDrone:
    def test_quadcopter_has_4_rotors(self):
        d = Drone(id=DroneId("d1"), name="quad")
        assert len(d.config.rotors) == 4

    def test_arm_disarm(self):
        d = Drone(id=DroneId("d1"), name="quad")
        d.arm()
        assert d.state.armed is True
        d.disarm()
        assert d.state.armed is False

    def test_compute_thrusts_when_disarmed(self):
        d = Drone(id=DroneId("d1"), name="quad")
        thrusts = d.compute_rotor_thrusts()
        assert all(t == 0.0 for t in thrusts)

    def test_compute_thrusts_hover(self):
        d = Drone(id=DroneId("d1"), name="quad")
        d.arm()
        d.set_command(thrust=0.5, roll=0, pitch=0, yaw_rate=0)
        thrusts = d.compute_rotor_thrusts()
        assert all(t > 0 for t in thrusts)
        assert len(thrusts) == 4


class TestMobileRobot:
    def test_differential_drive_step(self):
        r = MobileRobot(id=MobileRobotId("m1"), name="bot")
        r.set_velocity(linear_x=0.5, angular_z=0.0)
        r.step(0.1)
        assert r.state.pose.position.x > 0

    def test_velocity_clamping(self):
        r = MobileRobot(id=MobileRobotId("m1"), name="bot",
                        config=MobileRobotConfig(max_linear_speed=1.0))
        r.set_velocity(linear_x=5.0, angular_z=0.0)
        assert r.command.linear_x == 1.0

    def test_ackermann_steering(self):
        cfg = MobileRobotConfig(drive_type=DriveType.ACKERMANN)
        r = MobileRobot(id=MobileRobotId("m1"), name="car", config=cfg)
        r.set_velocity(linear_x=0.5, angular_z=0.5)
        r.step(0.1)
        assert r.state.heading != 0.0


class TestActuator:
    def test_position_control(self):
        a = Actuator(id=ActuatorId("a1"), name="a1", joint_name="j1",
                     control_mode=ControlMode.POSITION, kp=100, kd=10)
        a.set_target(1.0)
        effort = a.compute_effort(current_position=0.0, current_velocity=0.0, dt=0.01)
        assert effort > 0  # Should push toward target

    def test_effort_clamping(self):
        a = Actuator(id=ActuatorId("a1"), name="a1", joint_name="j1",
                     control_mode=ControlMode.EFFORT, max_effort=50.0)
        a.set_target(1000.0)
        effort = a.compute_effort(0, 0, 0.01)
        assert effort == 50.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
