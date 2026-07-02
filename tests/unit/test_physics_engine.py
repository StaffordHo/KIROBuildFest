"""Unit tests for physics engine — FK computation, joint control, URDF loading."""
import pytest
import math
import sys
sys.path.insert(0, ".")

from src.infrastructure.physics.simple_engine import SimplePhysicsEngine
from src.infrastructure.physics.urdf_loader import URDFLoader
from src.domain.models.robot import Robot, RobotId, RobotMetadata
from src.domain.models.joint import Joint, JointId, JointType, JointLimits
from src.domain.models.link import Link, LinkId
from src.domain.models.geometry import Vector3, Pose
from src.domain.models.world import World, WorldId
from src.interfaces.api.robot_factory import create_simple_arm


class TestSimplePhysicsEngine:
    def _make_engine_with_arm(self, dof=3):
        engine = SimplePhysicsEngine()
        world = World(id=WorldId("t"), name="test")
        engine.initialize(world)
        robot = create_simple_arm(name="test_arm", dof=dof, link_length=0.1)
        body_id = engine.load_robot(robot)
        return engine, body_id, robot

    def test_initialize(self):
        engine = SimplePhysicsEngine()
        world = World(id=WorldId("t"), name="test")
        engine.initialize(world)
        assert engine._dt == world.physics.time_step

    def test_load_robot(self):
        engine, body_id, robot = self._make_engine_with_arm()
        assert body_id == 1
        assert body_id in engine._robots

    def test_get_joint_states_initial(self):
        engine, body_id, robot = self._make_engine_with_arm()
        states = engine.get_joint_states(body_id)
        assert len(states) > 0
        for name, state in states.items():
            assert state.position == 0.0

    def test_set_joint_positions(self):
        engine, body_id, robot = self._make_engine_with_arm()
        engine.set_joint_positions(body_id, {"joint_1": 1.0})
        # Step to let PD controller act
        for _ in range(100):
            engine.step(1/240)
        states = engine.get_joint_states(body_id)
        # Should be approaching target
        assert abs(states["joint_1"].position - 1.0) < 0.5

    def test_fk_at_zero(self):
        engine, body_id, robot = self._make_engine_with_arm(dof=3)
        poses = engine.get_all_link_poses(body_id)
        # All links should have non-negative Z at zero config
        for name, (pos, orn) in poses.items():
            assert pos[2] >= 0.0

    def test_fk_changes_with_joint_angle(self):
        """Test that FK positions change when joints move.
        Uses a 6-DOF arm where joints have mixed axes."""
        engine, body_id, robot = self._make_engine_with_arm(dof=6)
        poses_zero = engine.get_all_link_poses(body_id)

        # Move multiple joints to ensure some offset changes
        engine.set_joint_positions(body_id, {
            "joint_1": 1.5, "joint_2": 1.0, "joint_3": -0.5
        })
        for _ in range(500):
            engine.step(1/240)
        poses_moved = engine.get_all_link_poses(body_id)

        # At least one link should have moved significantly
        # The factory arm alternates axes (Z, Y, X) so rotation causes displacement
        max_displacement = 0.0
        for name in poses_zero:
            if name in poses_moved:
                p0 = poses_zero[name][0]
                p1 = poses_moved[name][0]
                disp = sum((a-b)**2 for a, b in zip(p0, p1)) ** 0.5
                max_displacement = max(max_displacement, disp)

        assert max_displacement > 0.01, f"Max displacement was only {max_displacement}"

    def test_reset(self):
        engine, body_id, robot = self._make_engine_with_arm()
        engine.set_joint_positions(body_id, {"joint_1": 1.0})
        for _ in range(50):
            engine.step(1/240)
        engine.reset()
        states = engine.get_joint_states(body_id)
        for state in states.values():
            assert state.position == 0.0

    def test_rotation_matrix_identity(self):
        rot = SimplePhysicsEngine._rotation_matrix(0, 0, 1, 0)
        # Should be identity
        expected = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        for a, b in zip(rot, expected):
            assert abs(a - b) < 1e-10

    def test_rotation_matrix_90_degrees_z(self):
        rot = SimplePhysicsEngine._rotation_matrix(0, 0, 1, math.pi/2)
        # Rotating (1,0,0) around Z by 90deg should give (0,1,0)
        result = SimplePhysicsEngine._mat_vec(rot, [1, 0, 0])
        assert abs(result[0]) < 1e-10
        assert abs(result[1] - 1.0) < 1e-10
        assert abs(result[2]) < 1e-10


class TestURDFLoader:
    def test_load_simple_urdf(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial><mass value="1.0"/></inertial>
  </link>
  <link name="link1">
    <inertial><mass value="0.5"/></inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="2" effort="50"/>
  </joint>
</robot>"""
        loader = URDFLoader()
        robot = loader.load_from_string(urdf)
        assert robot.metadata.name == "test_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1
        assert robot.joints["joint1"].joint_type == JointType.REVOLUTE

    def test_parse_joint_limits(self):
        urdf = """<?xml version="1.0"?>
<robot name="test">
  <link name="a"><inertial><mass value="1"/></inertial></link>
  <link name="b"><inertial><mass value="1"/></inertial></link>
  <joint name="j1" type="revolute">
    <parent link="a"/><child link="b"/>
    <limit lower="-1.57" upper="1.57" velocity="3.0" effort="100"/>
  </joint>
</robot>"""
        loader = URDFLoader()
        robot = loader.load_from_string(urdf)
        limits = robot.joints["j1"].limits
        assert abs(limits.lower - (-1.57)) < 1e-10
        assert abs(limits.upper - 1.57) < 1e-10
        assert limits.velocity == 3.0
        assert limits.effort == 100.0

    def test_parse_fixed_joint(self):
        urdf = """<?xml version="1.0"?>
<robot name="test">
  <link name="a"><inertial><mass value="1"/></inertial></link>
  <link name="b"><inertial><mass value="1"/></inertial></link>
  <joint name="j1" type="fixed">
    <parent link="a"/><child link="b"/>
    <origin xyz="0.1 0.2 0.3"/>
  </joint>
</robot>"""
        loader = URDFLoader()
        robot = loader.load_from_string(urdf)
        j = robot.joints["j1"]
        assert j.joint_type == JointType.FIXED
        assert abs(j.origin.position.x - 0.1) < 1e-10
        assert abs(j.origin.position.y - 0.2) < 1e-10
        assert abs(j.origin.position.z - 0.3) < 1e-10


class TestRobotFactory:
    def test_create_simple_arm(self):
        robot = create_simple_arm(name="test", dof=6)
        assert robot.dof == 6
        assert len(robot.actuators) == 6
        assert "base_link" in robot.links
        assert "end_effector" in robot.links

    def test_arm_kinematic_chain(self):
        robot = create_simple_arm(name="test", dof=3)
        chain = robot.get_kinematic_chain("end_effector")
        # Chain should include joints leading to end effector
        assert len(chain) > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
