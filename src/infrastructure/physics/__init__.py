"""Physics engine infrastructure."""

from .simple_engine import SimplePhysicsEngine

try:
    from .pybullet_engine import PyBulletEngine
    HAS_PYBULLET = True
except ImportError:
    HAS_PYBULLET = False

__all__ = ["SimplePhysicsEngine"]
if HAS_PYBULLET:
    __all__.append("PyBulletEngine")
