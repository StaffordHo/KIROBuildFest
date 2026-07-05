"""Contact Physics — ground plane collision and basic object interaction.

Implements:
- Ground plane contact with spring-damper model
- Friction (Coulomb model)
- AABB collision detection between objects
- Contact force computation

This is lightweight enough to run in pure Python at 240Hz.
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ContactPoint:
    """A single contact between two bodies."""
    position: list[float]      # [x, y, z] world position
    normal: list[float]        # Contact normal (points from B to A)
    penetration: float         # Penetration depth (positive = overlap)
    body_a: str                # Name of body A
    body_b: str                # Name of body B (or "ground")


@dataclass
class ContactForce:
    """Resulting force from a contact."""
    force: list[float]         # [fx, fy, fz] in world frame
    torque: list[float]        # [tx, ty, tz] in world frame
    point: list[float]         # Application point


class GroundPlaneContact:
    """Spring-damper ground contact model.

    When a link's position goes below Z=0 (ground), apply an upward
    force proportional to penetration depth with damping.

    F_normal = k * penetration - b * velocity_z  (if penetration > 0)
    F_friction = -mu * |F_normal| * sign(v_tangential)
    """

    def __init__(
        self,
        stiffness: float = 5000.0,    # N/m — spring constant
        damping: float = 100.0,        # Ns/m — damping coefficient
        friction_mu: float = 0.8,      # Coulomb friction coefficient
        ground_z: float = 0.0,         # Ground plane height
    ):
        self.stiffness = stiffness
        self.damping = damping
        self.friction_mu = friction_mu
        self.ground_z = ground_z

    def compute_contact(
        self,
        link_position: list[float],
        link_velocity: list[float] = None,
        link_mass: float = 1.0,
    ) -> Optional[ContactForce]:
        """Compute ground contact force for a single link.

        Args:
            link_position: [x, y, z] position of link center
            link_velocity: [vx, vy, vz] velocity (or None for static)
            link_mass: Mass of the link (for gravity compensation)

        Returns:
            ContactForce if in contact, None otherwise.
        """
        z = link_position[2]
        penetration = self.ground_z - z

        if penetration <= 0:
            return None  # Not in contact

        # Normal force (spring-damper)
        vz = link_velocity[2] if link_velocity else 0.0
        f_normal = self.stiffness * penetration - self.damping * vz
        f_normal = max(0.0, f_normal)  # Only push up, never pull down

        # Friction force (Coulomb model)
        fx, fy = 0.0, 0.0
        if link_velocity:
            vx = link_velocity[0]
            vy = link_velocity[1]
            v_tangential = math.sqrt(vx * vx + vy * vy)
            if v_tangential > 0.001:
                f_friction = self.friction_mu * f_normal
                fx = -f_friction * (vx / v_tangential)
                fy = -f_friction * (vy / v_tangential)

        return ContactForce(
            force=[fx, fy, f_normal],
            torque=[0.0, 0.0, 0.0],
            point=[link_position[0], link_position[1], self.ground_z],
        )


class AABBCollisionDetector:
    """Axis-Aligned Bounding Box collision detection.

    Checks overlap between rectangular bounding boxes.
    Used for robot-object and object-object collision.
    """

    @staticmethod
    def check_overlap(
        pos_a: list[float], size_a: list[float],
        pos_b: list[float], size_b: list[float],
    ) -> Optional[ContactPoint]:
        """Check if two AABBs overlap.

        Args:
            pos_a: Center position of box A [x, y, z]
            size_a: Half-extents of box A [hx, hy, hz]
            pos_b: Center position of box B [x, y, z]
            size_b: Half-extents of box B [hx, hy, hz]

        Returns:
            ContactPoint if overlapping, None otherwise.
        """
        # Check overlap on each axis
        for i in range(3):
            dist = abs(pos_a[i] - pos_b[i])
            overlap = (size_a[i] + size_b[i]) - dist
            if overlap <= 0:
                return None  # No overlap on this axis = no collision

        # Find minimum penetration axis
        min_penetration = float('inf')
        min_axis = 0
        for i in range(3):
            dist = abs(pos_a[i] - pos_b[i])
            overlap = (size_a[i] + size_b[i]) - dist
            if overlap < min_penetration:
                min_penetration = overlap
                min_axis = i

        # Contact normal points from B to A along minimum penetration axis
        normal = [0.0, 0.0, 0.0]
        normal[min_axis] = 1.0 if pos_a[min_axis] > pos_b[min_axis] else -1.0

        # Contact point at midpoint between surfaces
        contact_pos = [
            (pos_a[0] + pos_b[0]) / 2,
            (pos_a[1] + pos_b[1]) / 2,
            (pos_a[2] + pos_b[2]) / 2,
        ]

        return ContactPoint(
            position=contact_pos,
            normal=normal,
            penetration=min_penetration,
            body_a="a",
            body_b="b",
        )

    @staticmethod
    def resolve_penetration(
        contact: ContactPoint,
        mass_a: float = 1.0,
        mass_b: float = float('inf'),  # infinite mass = static
        restitution: float = 0.3,
    ) -> tuple[list[float], list[float]]:
        """Compute position corrections to resolve penetration.

        Returns:
            (correction_a, correction_b) — displacement vectors.
        """
        total_inv_mass = (1.0 / mass_a) + (1.0 / mass_b if mass_b < float('inf') else 0.0)
        if total_inv_mass == 0:
            return [0, 0, 0], [0, 0, 0]

        correction_mag = contact.penetration / total_inv_mass

        correction_a = [
            contact.normal[i] * correction_mag / mass_a
            for i in range(3)
        ]
        correction_b = [
            -contact.normal[i] * correction_mag / mass_b if mass_b < float('inf') else 0.0
            for i in range(3)
        ]

        return correction_a, correction_b


class ContactWorld:
    """Manages all contacts in the simulation world.

    Applies ground contact forces to all links and detects
    collisions between objects.
    """

    def __init__(self):
        self.ground = GroundPlaneContact()
        self.aabb = AABBCollisionDetector()
        self.contacts: list[ContactPoint] = []

    def step(self, link_poses: dict[str, tuple], dt: float) -> dict[str, list[float]]:
        """Compute contact forces for all links.

        Args:
            link_poses: {link_name: ([x,y,z], [qx,qy,qz,qw])}
            dt: Time step

        Returns:
            {link_name: [fx, fy, fz]} — forces to apply
        """
        self.contacts.clear()
        forces = {}

        for link_name, (position, _) in link_poses.items():
            # Ground contact
            contact_force = self.ground.compute_contact(
                link_position=position,
                link_velocity=[0, 0, 0],  # Simplified: no velocity tracking per-link
                link_mass=1.0,
            )
            if contact_force:
                forces[link_name] = contact_force.force
                self.contacts.append(ContactPoint(
                    position=contact_force.point,
                    normal=[0, 0, 1],
                    penetration=self.ground.ground_z - position[2],
                    body_a=link_name,
                    body_b="ground",
                ))

        return forces

    @property
    def num_contacts(self) -> int:
        return len(self.contacts)

    def get_contact_points(self) -> list[dict]:
        """Get contact points for visualization."""
        return [
            {
                "position": c.position,
                "normal": c.normal,
                "penetration": c.penetration,
                "body_a": c.body_a,
                "body_b": c.body_b,
            }
            for c in self.contacts
        ]
