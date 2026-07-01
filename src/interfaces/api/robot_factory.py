"""Robot factory — creates common robot configurations programmatically.

Provides quick-start robots for testing without needing URDF files.
"""

import uuid
from ...domain.models.robot import Robot, RobotId, RobotMetadata
from ...domain.models.joint import Joint, JointId, JointType, JointLimits
from ...domain.models.link import Link, LinkId, Inertial, Visual, Collision
from ...domain.models.actuator import Actuator, ActuatorId, ControlMode
from ...domain.models.geometry import Vector3, Pose, Quaternion, Box, Cylinder


def create_simple_arm(name: str = "simple_arm", dof: int = 6, link_length: float = 0.1) -> Robot:
    """Create a simple serial manipulator arm.

    Args:
        name: Robot name.
        dof: Number of joints (degrees of freedom).
        link_length: Length of each link segment (meters).

    Returns:
        A fully configured Robot with N revolute joints.
    """
    robot = Robot(
        id=RobotId(str(uuid.uuid4())),
        metadata=RobotMetadata(
            name=name,
            description=f"{dof}-DOF serial manipulator",
            author="RoboSim Factory",
            version="1.0.0",
            source_format="generated",
        ),
    )

    # Base link (fixed to world)
    base_link = Link(
        id=LinkId("base_link"),
        name="base_link",
        inertial=Inertial(mass=10.0, ixx=0.1, iyy=0.1, izz=0.1),
        visuals=[Visual(
            name="base_visual",
            geometry=Cylinder(radius=0.05, length=0.02),
            color=(0.3, 0.3, 0.3, 1.0),
        )],
        collisions=[Collision(
            name="base_collision",
            geometry=Cylinder(radius=0.05, length=0.02),
        )],
    )
    robot.add_link(base_link)

    # Create chain of links and joints
    prev_link_name = "base_link"
    link_mass = 1.0

    for i in range(dof):
        link_name = f"link_{i+1}"
        joint_name = f"joint_{i+1}"

        # Alternate rotation axes for interesting motion
        if i % 3 == 0:
            axis = Vector3(0, 0, 1)  # Z-axis (yaw)
        elif i % 3 == 1:
            axis = Vector3(0, 1, 0)  # Y-axis (pitch)
        else:
            axis = Vector3(1, 0, 0)  # X-axis (roll)

        # Create link
        link = Link(
            id=LinkId(link_name),
            name=link_name,
            inertial=Inertial(
                mass=link_mass,
                ixx=link_mass * link_length**2 / 12,
                iyy=link_mass * link_length**2 / 12,
                izz=link_mass * 0.01**2 / 2,
            ),
            visuals=[Visual(
                name=f"{link_name}_visual",
                geometry=Cylinder(radius=0.02, length=link_length),
                color=_get_link_color(i, dof),
            )],
            collisions=[Collision(
                name=f"{link_name}_collision",
                geometry=Cylinder(radius=0.02, length=link_length),
            )],
        )
        robot.add_link(link)

        # Create joint
        joint = Joint(
            id=JointId(joint_name),
            name=joint_name,
            joint_type=JointType.REVOLUTE,
            parent_link=prev_link_name,
            child_link=link_name,
            origin=Pose(
                position=Vector3(0, 0, link_length if i > 0 else 0.05),
                orientation=Quaternion.identity(),
            ),
            axis=axis,
            limits=JointLimits(
                lower=-3.14159,
                upper=3.14159,
                velocity=2.0,
                effort=50.0,
            ),
            damping=0.1,
        )
        robot.add_joint(joint)

        # Create actuator
        actuator = Actuator(
            id=ActuatorId(f"actuator_{i+1}"),
            name=f"actuator_{i+1}",
            joint_name=joint_name,
            control_mode=ControlMode.POSITION,
            kp=100.0,
            kd=10.0,
            max_effort=50.0,
        )
        robot.add_actuator(actuator)

        prev_link_name = link_name
        link_mass *= 0.8  # Decreasing mass toward end effector

    # End effector link
    ee_link = Link(
        id=LinkId("end_effector"),
        name="end_effector",
        inertial=Inertial(mass=0.1),
        visuals=[Visual(
            name="ee_visual",
            geometry=Box(0.04, 0.04, 0.02),
            color=(1.0, 0.2, 0.2, 1.0),
        )],
        collisions=[Collision(
            name="ee_collision",
            geometry=Box(0.04, 0.04, 0.02),
        )],
    )
    robot.add_link(ee_link)

    # Fixed joint to end effector
    ee_joint = Joint(
        id=JointId("ee_joint"),
        name="ee_joint",
        joint_type=JointType.FIXED,
        parent_link=prev_link_name,
        child_link="end_effector",
        origin=Pose(position=Vector3(0, 0, link_length / 2)),
    )
    robot.add_joint(ee_joint)

    return robot


def _get_link_color(index: int, total: int) -> tuple:
    """Generate a color gradient for robot links."""
    t = index / max(total - 1, 1)
    # Blue to orange gradient
    r = 0.2 + t * 0.8
    g = 0.4 + t * 0.2
    b = 0.9 - t * 0.7
    return (r, g, b, 1.0)
