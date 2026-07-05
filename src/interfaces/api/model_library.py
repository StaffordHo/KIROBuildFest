"""Model Library — curated catalog of open-source robot URDF models.

Enables one-click loading of popular robots from public repositories.
Users can also submit their own models or fetch from any GitHub URL.

Enterprise ontology:
- ModelCatalog → ModelEntry → ModelVersion
- Supports tagging, categorization, and validation status
"""

from __future__ import annotations
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional
from enum import Enum
import httpx
import uuid

router = APIRouter(prefix="/models", tags=["Model Library"])


# ============================================================================
# ONTOLOGY
# ============================================================================

class RobotCategory(str, Enum):
    """Enterprise classification of robot systems."""
    MANIPULATOR = "manipulator"          # Industrial/research arms
    MOBILE_GROUND = "mobile_ground"      # Wheeled/tracked ground robots
    AERIAL = "aerial"                    # Drones, VTOL
    LEGGED = "legged"                    # Bipedal, quadruped
    HUMANOID = "humanoid"               # Full humanoid
    COLLABORATIVE = "collaborative"      # Cobots (human-safe)
    UNDERWATER = "underwater"            # ROV, AUV
    SPACE = "space"                      # Space manipulators
    CUSTOM = "custom"                    # User-defined


class ValidationStatus(str, Enum):
    """Model validation state for enterprise use."""
    UNVERIFIED = "unverified"            # Just uploaded, not tested
    KINEMATICS_OK = "kinematics_ok"      # FK/IK verified
    DYNAMICS_OK = "dynamics_ok"          # Mass/inertia verified
    PRODUCTION_READY = "production_ready"  # Full validation passed


class ModelEntry(BaseModel):
    """A robot model in the catalog."""
    id: str
    name: str
    description: str
    category: RobotCategory
    manufacturer: str = ""
    dof: int = 0
    source_url: str                      # Raw URDF download URL
    thumbnail_url: str = ""
    tags: list[str] = []
    validation_status: ValidationStatus = ValidationStatus.UNVERIFIED
    payload_kg: float = 0.0             # Max payload (for arms)
    reach_m: float = 0.0                # Max reach (for arms)
    weight_kg: float = 0.0             # Robot weight
    license: str = "open-source"


# ============================================================================
# CURATED CATALOG — Popular open-source robots
# ============================================================================

MODEL_CATALOG: list[ModelEntry] = [
    # --- MANIPULATORS ---
    ModelEntry(
        id="panda_franka",
        name="Franka Emika Panda",
        description="9-DOF collaborative research arm with gripper. Industry standard for manipulation research.",
        category=RobotCategory.MANIPULATOR,
        manufacturer="Franka Emika",
        dof=9,
        source_url="https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/gym/pybullet_data/franka_panda/panda.urdf",
        tags=["cobot", "research", "7dof", "gripper", "torque-controlled"],
        payload_kg=3.0,
        reach_m=0.855,
        weight_kg=18.0,
        license="Apache-2.0",
    ),
    ModelEntry(
        id="kuka_iiwa",
        name="KUKA iiwa 14",
        description="7-DOF lightweight industrial robot with torque sensors in every joint.",
        category=RobotCategory.COLLABORATIVE,
        manufacturer="KUKA",
        dof=7,
        source_url="https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/gym/pybullet_data/kuka_iiwa/model.urdf",
        tags=["industrial", "collaborative", "7dof", "sensitive"],
        payload_kg=14.0,
        reach_m=0.820,
        weight_kg=29.9,
        license="BSD",
    ),

    # --- MOBILE GROUND ---
    ModelEntry(
        id="turtlebot3",
        name="TurtleBot3 Burger",
        description="Compact differential drive robot. ROS2 reference platform for education.",
        category=RobotCategory.MOBILE_GROUND,
        manufacturer="ROBOTIS",
        dof=2,
        source_url="https://raw.githubusercontent.com/Daniella1/urdf_files_dataset/main/urdf_files/oems/xacro_generated/turtlebot3_robotis/turtlebot3_description/urdf/turtlebot3_burger.urdf",
        tags=["education", "ros2", "differential", "compact"],
        weight_kg=1.0,
        license="Apache-2.0",
    ),
    ModelEntry(
        id="jackal",
        name="Clearpath Jackal",
        description="Rugged outdoor UGV with skid-steer drive. Common in field robotics research.",
        category=RobotCategory.MOBILE_GROUND,
        manufacturer="Clearpath Robotics",
        dof=4,
        source_url="https://raw.githubusercontent.com/Daniella1/urdf_files_dataset/main/urdf_files/oems/xacro_generated/jackal_clearpath_robotics/jackal_description/urdf/jackal.urdf",
        tags=["outdoor", "rugged", "field-robotics", "skid-steer"],
        payload_kg=20.0,
        weight_kg=17.0,
        license="BSD",
    ),
    ModelEntry(
        id="racecar",
        name="MIT Racecar",
        description="Ackermann-steered RC car platform for autonomous driving research.",
        category=RobotCategory.MOBILE_GROUND,
        manufacturer="MIT",
        dof=6,
        source_url="https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/gym/pybullet_data/racecar/racecar.urdf",
        tags=["ackermann", "autonomous-driving", "racing", "education"],
        weight_kg=3.5,
        license="BSD",
    ),
    ModelEntry(
        id="r2d2",
        name="R2D2 Robot",
        description="Articulated mobile robot. Great for testing basic simulation and joint control.",
        category=RobotCategory.MOBILE_GROUND,
        manufacturer="PyBullet",
        dof=8,
        source_url="https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/gym/pybullet_data/r2d2.urdf",
        tags=["education", "classic", "mobile", "articulated"],
        weight_kg=5.0,
        license="BSD",
    ),

    # --- LEGGED ---
    ModelEntry(
        id="a1",
        name="Unitree A1",
        description="12-DOF quadruped robot for agile locomotion research. High-performance legged platform.",
        category=RobotCategory.LEGGED,
        manufacturer="Unitree",
        dof=12,
        source_url="https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/gym/pybullet_data/a1/a1.urdf",
        tags=["quadruped", "agile", "locomotion", "research", "dynamic"],
        weight_kg=12.0,
        license="BSD",
    ),
    ModelEntry(
        id="laikago",
        name="Unitree Laikago",
        description="12-DOF quadruped robot. Predecessor to A1, widely used in sim-to-real research.",
        category=RobotCategory.LEGGED,
        manufacturer="Unitree",
        dof=12,
        source_url="https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/gym/pybullet_data/laikago/laikago.urdf",
        tags=["quadruped", "locomotion", "sim-to-real", "research"],
        weight_kg=22.0,
        license="BSD",
    ),
    ModelEntry(
        id="minitaur",
        name="Ghost Robotics Minitaur",
        description="16-DOF compact quadruped with direct-drive legs. Benchmark for legged locomotion control.",
        category=RobotCategory.LEGGED,
        manufacturer="Ghost Robotics",
        dof=16,
        source_url="https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/gym/pybullet_data/quadruped/minitaur.urdf",
        tags=["quadruped", "direct-drive", "compact", "benchmark"],
        weight_kg=6.0,
        license="BSD",
    ),

    # --- BENCHMARKS ---
    ModelEntry(
        id="cartpole",
        name="Cart-Pole System",
        description="Classic control benchmark: balance a pole on a moving cart. Ideal for RL testing.",
        category=RobotCategory.CUSTOM,
        manufacturer="PyBullet",
        dof=2,
        source_url="https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/gym/pybullet_data/cartpole.urdf",
        tags=["control", "benchmark", "reinforcement-learning", "classic"],
        weight_kg=2.0,
        license="BSD",
    ),

    # --- HUMANOID / BIPEDAL ---
    ModelEntry(
        id="humanoid",
        name="Humanoid (21-DOF)",
        description="Full humanoid bipedal model with 34 links. Suitable for whole-body control and locomotion research.",
        category=RobotCategory.HUMANOID,
        manufacturer="PyBullet",
        dof=21,
        source_url="https://raw.githubusercontent.com/bulletphysics/bullet3/master/data/humanoid.urdf",
        tags=["humanoid", "bipedal", "locomotion", "whole-body", "research"],
        weight_kg=75.0,
        license="BSD",
    ),
    ModelEntry(
        id="cassie",
        name="Agility Robotics Cassie",
        description="14-DOF bipedal robot for dynamic walking and running. Key platform in sim-to-real locomotion research.",
        category=RobotCategory.LEGGED,
        manufacturer="Agility Robotics",
        dof=14,
        source_url="https://raw.githubusercontent.com/UMich-BipedLab/cassie_description/master/urdf/cassie.urdf",
        tags=["bipedal", "dynamic-walking", "running", "sim-to-real"],
        weight_kg=31.0,
        license="MIT",
    ),
    ModelEntry(
        id="atlas",
        name="Atlas Humanoid (30-DOF)",
        description="High-DOF humanoid for advanced bipedal locomotion and manipulation. Full upper and lower body.",
        category=RobotCategory.HUMANOID,
        manufacturer="Boston Dynamics (model)",
        dof=30,
        source_url="https://raw.githubusercontent.com/openai/roboschool/master/roboschool/models_robot/atlas_description/urdf/atlas_v4_with_multisense.urdf",
        tags=["humanoid", "bipedal", "high-dof", "locomotion", "manipulation"],
        weight_kg=80.0,
        license="BSD",
    ),
]


# ============================================================================
# API ENDPOINTS
# ============================================================================

class FetchModelRequest(BaseModel):
    """Request to fetch a model from the catalog or a URL."""
    model_id: Optional[str] = None       # From catalog
    url: Optional[str] = None            # Direct URL to URDF
    world_id: str                         # Target world to load into


class ModelSearchRequest(BaseModel):
    category: Optional[RobotCategory] = None
    search: Optional[str] = None
    tags: Optional[list[str]] = None


@router.get("/catalog")
async def get_catalog():
    """Get the full model catalog."""
    return {
        "models": [m.dict() for m in MODEL_CATALOG],
        "total": len(MODEL_CATALOG),
        "categories": [c.value for c in RobotCategory],
    }


@router.get("/catalog/{model_id}")
async def get_model_details(model_id: str):
    """Get details for a specific model."""
    model = next((m for m in MODEL_CATALOG if m.id == model_id), None)
    if not model:
        raise HTTPException(404, f"Model '{model_id}' not found in catalog")
    return model.dict()


@router.post("/catalog/search")
async def search_models(request: ModelSearchRequest):
    """Search/filter the model catalog."""
    results = MODEL_CATALOG

    if request.category:
        results = [m for m in results if m.category == request.category]

    if request.search:
        q = request.search.lower()
        results = [m for m in results if (
            q in m.name.lower() or
            q in m.description.lower() or
            q in m.manufacturer.lower() or
            any(q in tag for tag in m.tags)
        )]

    if request.tags:
        results = [m for m in results if any(t in m.tags for t in request.tags)]

    return {"models": [m.dict() for m in results], "total": len(results)}


@router.post("/fetch")
async def fetch_and_load_model(request: FetchModelRequest):
    """Fetch a URDF from catalog or URL and load it into a world.

    This is the core "one-click deploy" endpoint.
    """
    urdf_url = None

    if request.model_id:
        model = next((m for m in MODEL_CATALOG if m.id == request.model_id), None)
        if not model:
            raise HTTPException(404, f"Model '{request.model_id}' not found")
        urdf_url = model.source_url
    elif request.url:
        urdf_url = request.url
    else:
        raise HTTPException(400, "Provide either model_id or url")

    # Fetch URDF content from remote URL
    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.get(urdf_url)
            response.raise_for_status()
            urdf_content = response.text
    except httpx.HTTPError as e:
        raise HTTPException(502, f"Failed to fetch URDF: {str(e)}")

    # Validate it's actually XML/URDF
    if "<robot" not in urdf_content and "<link" not in urdf_content:
        raise HTTPException(422, "Fetched content doesn't appear to be a valid URDF file")

    # Parse into domain model
    from ...infrastructure.physics.urdf_loader import URDFLoader
    loader = URDFLoader()

    try:
        robot = loader.load_from_string(urdf_content)
    except Exception as e:
        raise HTTPException(422, f"Failed to parse URDF: {str(e)}")

    # Load into the specified world
    from .main import _worlds, _physics_engines
    world = _worlds.get(request.world_id)
    if not world:
        raise HTTPException(404, f"World '{request.world_id}' not found")

    world.add_robot(robot)
    engine = _physics_engines.get(request.world_id)
    if engine:
        body_id = engine.load_robot(robot)

        # Auto-elevate: if any link FK position goes below Z=0, offset the base
        poses = engine.get_all_link_poses(body_id)
        if poses:
            min_z = min(p[0][2] for p in poses.values())
            if min_z < -0.01:
                # Need to elevate - adjust all joint targets won't work,
                # so we shift the robot's stored base offset
                elevation = -min_z + 0.03
                robot_state = engine._robots.get(body_id)
                if robot_state:
                    from ...domain.models.geometry import Vector3, Pose
                    robot_state.robot.base_pose = Pose(
                        position=Vector3(0, 0, elevation)
                    )

    return {
        "success": True,
        "robot_id": str(robot.id),
        "name": robot.metadata.name,
        "dof": robot.dof,
        "links": len(robot.links),
        "joints": list(robot.joints.keys()),
        "source": urdf_url,
        "mesh_base_url": urdf_url.rsplit("/", 1)[0] + "/",  # Directory containing the URDF
    }


@router.get("/categories")
async def list_categories():
    """List all robot categories with counts."""
    counts = {}
    for model in MODEL_CATALOG:
        cat = model.category.value
        counts[cat] = counts.get(cat, 0) + 1

    return {
        "categories": [
            {"name": c.value, "count": counts.get(c.value, 0), "label": c.value.replace("_", " ").title()}
            for c in RobotCategory
        ]
    }
