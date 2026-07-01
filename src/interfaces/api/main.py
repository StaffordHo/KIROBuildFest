"""FastAPI application — REST + WebSocket interface for RoboSim."""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from contextlib import asynccontextmanager
import asyncio
import json
import uuid
import tempfile
import os
from pathlib import Path

from ...domain.models.world import World, WorldId, PhysicsConfig, SimStatus, StaticObject, DynamicObject
from ...domain.models.robot import Robot, RobotId, RobotMetadata
from ...domain.models.geometry import Vector3, Pose
from ...domain.services.simulation_service import SimulationService
from ...infrastructure.physics.simple_engine import SimplePhysicsEngine
from ...infrastructure.physics.urdf_loader import URDFLoader
from .schemas import (
    WorldCreateRequest,
    RobotCreateRequest,
    JointCommandRequest,
    SimControlRequest,
    WorldStateResponse,
)

# Try to import PyBullet engine
try:
    from ...infrastructure.physics.pybullet_engine import PyBulletEngine
    _USE_PYBULLET = True
except ImportError:
    _USE_PYBULLET = False


# --- Application State ---
_worlds: dict[str, World] = {}
_sim_services: dict[str, SimulationService] = {}
_physics_engines: dict[str, object] = {}  # Engine instances (PyBullet or Simple)
_sim_tasks: dict[str, asyncio.Task] = {}
_ws_connections: dict[str, list[WebSocket]] = {}  # world_id -> connected clients


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan — startup and shutdown."""
    print("🤖 RoboSim server starting...")
    yield
    # Cleanup
    print("Shutting down simulation engines...")
    for engine in _physics_engines.values():
        engine.shutdown()
    for task in _sim_tasks.values():
        task.cancel()


app = FastAPI(
    title="RoboSim",
    description="Open Robotics Simulation Platform — API",
    version="0.1.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Restrict in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve static files (frontend)
_static_dir = Path(__file__).parent.parent.parent.parent / "static"
if _static_dir.exists():
    app.mount("/static", StaticFiles(directory=str(_static_dir)), name="static")


@app.get("/")
async def root():
    """Serve the frontend."""
    index_path = _static_dir / "index.html"
    if index_path.exists():
        return FileResponse(str(index_path))
    return {"message": "RoboSim API. Visit /docs for API documentation."}


# Register robot types router
from .robot_types import router as robot_types_router, step_all as step_all_robots
app.include_router(robot_types_router, prefix="/api", tags=["Robot Types"])

# Register model library router
from .model_library import router as model_library_router
app.include_router(model_library_router, prefix="/api", tags=["Model Library"])


# --- REST Endpoints ---

@app.get("/health")
async def health():
    return {"status": "ok", "active_worlds": len(_worlds)}


@app.post("/worlds", status_code=201)
async def create_world(request: WorldCreateRequest):
    """Create a new simulation world."""
    world_id = WorldId(str(uuid.uuid4()))
    physics_config = PhysicsConfig(
        gravity=Vector3(0, 0, request.gravity_z),
        time_step=request.time_step,
    )

    world = World(id=world_id, name=request.name, physics=physics_config)

    # Add ground plane as static object
    world.add_static_object(StaticObject(name="ground_plane"))

    # Initialize physics engine (prefer PyBullet, fall back to simple)
    if _USE_PYBULLET:
        engine = PyBulletEngine()
    else:
        engine = SimplePhysicsEngine()
    engine.initialize(world)

    sim_service = SimulationService(engine)

    _worlds[str(world_id)] = world
    _physics_engines[str(world_id)] = engine
    _sim_services[str(world_id)] = sim_service
    _ws_connections[str(world_id)] = []

    return {
        "world_id": str(world_id),
        "name": world.name,
        "status": world.state.status.value,
    }


@app.get("/worlds")
async def list_worlds():
    """List all active simulation worlds."""
    return [
        {
            "world_id": wid,
            "name": world.name,
            "status": world.state.status.value,
            "robots": len(world.robots),
            "sim_time": world.state.sim_time,
        }
        for wid, world in _worlds.items()
    ]


@app.get("/worlds/{world_id}")
async def get_world(world_id: str):
    """Get detailed world state."""
    world = _worlds.get(world_id)
    if not world:
        raise HTTPException(404, "World not found")

    return {
        "world_id": world_id,
        "name": world.name,
        "status": world.state.status.value,
        "sim_time": world.state.sim_time,
        "step_count": world.state.step_count,
        "physics": {
            "gravity_z": world.physics.gravity.z,
            "time_step": world.physics.time_step,
        },
        "robots": {
            rid: {
                "name": r.metadata.name,
                "dof": r.dof,
                "joints": list(r.joints.keys()),
            }
            for rid, r in world.robots.items()
        },
    }


@app.post("/worlds/{world_id}/robots/upload")
async def upload_robot_urdf(world_id: str, file: UploadFile = File(...)):
    """Upload a URDF file to add a robot to the world."""
    world = _worlds.get(world_id)
    if not world:
        raise HTTPException(404, "World not found")

    content = await file.read()
    urdf_string = content.decode("utf-8")

    loader = URDFLoader()
    robot = loader.load_from_string(urdf_string, name=file.filename or "uploaded")

    world.add_robot(robot)

    # Load into physics engine
    engine = _physics_engines[world_id]

    if _USE_PYBULLET:
        # Write temp file for PyBullet (it needs a file path)
        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
            f.write(urdf_string)
            temp_path = f.name
        try:
            physics_id = engine.load_robot_from_urdf(temp_path)
            engine._robot_bodies[physics_id]["robot_id"] = str(robot.id)
        finally:
            os.unlink(temp_path)
    else:
        # Simple engine can load directly from domain model
        physics_id = engine.load_robot(robot)

    return {
        "robot_id": str(robot.id),
        "name": robot.metadata.name,
        "dof": robot.dof,
        "links": list(robot.links.keys()),
        "joints": list(robot.joints.keys()),
    }


@app.post("/worlds/{world_id}/robots")
async def create_robot(world_id: str, request: RobotCreateRequest):
    """Create a simple robot from parameters (for quick testing)."""
    world = _worlds.get(world_id)
    if not world:
        raise HTTPException(404, "World not found")

    # This creates a simple N-DOF arm programmatically
    from .robot_factory import create_simple_arm
    robot = create_simple_arm(
        name=request.name,
        dof=request.dof,
        link_length=request.link_length,
    )

    world.add_robot(robot)

    # Load into physics
    engine = _physics_engines[world_id]
    physics_id = engine.load_robot(robot)

    return {
        "robot_id": str(robot.id),
        "name": robot.metadata.name,
        "dof": robot.dof,
        "joints": list(robot.joints.keys()),
    }


@app.post("/worlds/{world_id}/control")
async def simulation_control(world_id: str, request: SimControlRequest):
    """Start, pause, stop, or reset the simulation."""
    world = _worlds.get(world_id)
    if not world:
        raise HTTPException(404, "World not found")

    if request.action == "start":
        world.start()
        # Start simulation loop task
        if world_id not in _sim_tasks or _sim_tasks[world_id].done():
            _sim_tasks[world_id] = asyncio.create_task(_simulation_loop(world_id))
    elif request.action == "pause":
        world.pause()
    elif request.action == "stop":
        world.stop()
        if world_id in _sim_tasks:
            _sim_tasks[world_id].cancel()
    elif request.action == "reset":
        sim = _sim_services.get(world_id)
        if sim:
            sim.reset(world)
    else:
        raise HTTPException(400, f"Unknown action: {request.action}")

    return {"status": world.state.status.value, "sim_time": world.state.sim_time}


@app.post("/worlds/{world_id}/joints")
async def set_joint_commands(world_id: str, request: JointCommandRequest):
    """Send joint position/velocity commands to a robot."""
    world = _worlds.get(world_id)
    if not world:
        raise HTTPException(404, "World not found")

    robot = None
    for r in world.robots.values():
        if str(r.id) == request.robot_id or r.metadata.name == request.robot_id:
            robot = r
            break

    if not robot:
        raise HTTPException(404, f"Robot '{request.robot_id}' not found")

    # Apply position commands via physics engine
    engine = _physics_engines[world_id]

    if _USE_PYBULLET:
        for body_id, info in engine._robot_bodies.items():
            if info["robot_id"] == str(robot.id):
                engine.set_joint_positions(body_id, request.positions)
                break
    else:
        # Simple engine: find the robot by ID
        for body_id, robot_state in engine._robots.items():
            if robot_state.robot_id == str(robot.id):
                engine.set_joint_positions(body_id, request.positions)
                break

    return {"applied": request.positions}


@app.get("/worlds/{world_id}/robots/{robot_id}/geometry")
async def get_robot_geometry(world_id: str, robot_id: str):
    """Get robot geometry data for 3D visualization.

    Returns link dimensions, joint origins, and structure needed
    to render the robot without mesh files.
    """
    world = _worlds.get(world_id)
    if not world:
        raise HTTPException(404, "World not found")

    robot = None
    for r in world.robots.values():
        if str(r.id) == robot_id or r.metadata.name == robot_id:
            robot = r
            break

    if not robot:
        raise HTTPException(404, f"Robot '{robot_id}' not found")

    from ...domain.models.geometry import Box, Cylinder, Sphere

    links_data = {}
    for link_name, link in robot.links.items():
        link_info = {"name": link_name, "visuals": [], "collisions": []}

        for vis in link.visuals:
            geom_data = _serialize_geometry(vis.geometry)
            if geom_data:
                geom_data["color"] = list(vis.color) if vis.color else [0.5, 0.5, 0.5, 1.0]
                link_info["visuals"].append(geom_data)

        for col in link.collisions:
            geom_data = _serialize_geometry(col.geometry)
            if geom_data:
                link_info["collisions"].append(geom_data)

        links_data[link_name] = link_info

    joints_data = {}
    for joint_name, joint in robot.joints.items():
        joints_data[joint_name] = {
            "parent": joint.parent_link,
            "child": joint.child_link,
            "type": joint.joint_type.value,
            "origin": {
                "x": joint.origin.position.x,
                "y": joint.origin.position.y,
                "z": joint.origin.position.z,
            },
            "axis": {"x": joint.axis.x, "y": joint.axis.y, "z": joint.axis.z},
        }

    return {
        "robot_id": str(robot.id),
        "name": robot.metadata.name,
        "links": links_data,
        "joints": joints_data,
    }


def _serialize_geometry(geom) -> dict:
    """Convert geometry to JSON-serializable dict."""
    from ...domain.models.geometry import Box, Cylinder, Sphere, Mesh
    if geom is None:
        return None
    if isinstance(geom, Box):
        return {"type": "box", "size_x": geom.size_x, "size_y": geom.size_y, "size_z": geom.size_z}
    elif isinstance(geom, Cylinder):
        return {"type": "cylinder", "radius": geom.radius, "length": geom.length}
    elif isinstance(geom, Sphere):
        return {"type": "sphere", "radius": geom.radius}
    elif isinstance(geom, Mesh):
        return {"type": "mesh", "filename": geom.filename}
    return None


# --- WebSocket for real-time state streaming ---

@app.websocket("/ws/{world_id}")
async def websocket_endpoint(websocket: WebSocket, world_id: str):
    """WebSocket for real-time simulation state streaming."""
    if world_id not in _worlds:
        await websocket.close(code=4004, reason="World not found")
        return

    await websocket.accept()
    _ws_connections[world_id].append(websocket)

    try:
        while True:
            # Receive commands from client
            data = await websocket.receive_text()
            msg = json.loads(data)

            if msg.get("type") == "joint_command":
                # Apply joint commands
                engine = _physics_engines.get(world_id)
                if engine:
                    positions = msg.get("positions", {})
                    if _USE_PYBULLET:
                        for body_id in engine._robot_bodies:
                            engine.set_joint_positions(body_id, positions)
                    else:
                        for body_id in engine._robots:
                            engine.set_joint_positions(body_id, positions)

            elif msg.get("type") == "sim_control":
                world = _worlds[world_id]
                action = msg.get("action")
                if action == "start":
                    world.start()
                    if world_id not in _sim_tasks or _sim_tasks[world_id].done():
                        _sim_tasks[world_id] = asyncio.create_task(_simulation_loop(world_id))
                elif action == "pause":
                    world.pause()
                elif action == "step":
                    # Single step
                    sim = _sim_services.get(world_id)
                    if sim:
                        sim.step(world)

    except WebSocketDisconnect:
        _ws_connections[world_id].remove(websocket)


# --- Simulation Loop ---

async def _simulation_loop(world_id: str):
    """Background task that runs the simulation and streams state."""
    world = _worlds.get(world_id)
    sim = _sim_services.get(world_id)
    engine = _physics_engines.get(world_id)

    if not all([world, sim, engine]):
        return

    target_dt = world.physics.time_step
    stream_rate = 1.0 / 60.0  # Stream at 60 Hz

    steps_per_stream = max(1, int(stream_rate / target_dt))
    step_counter = 0

    while world.is_running:
        # Step arm physics
        sim.step(world)

        # Step drones + mobile robots + tasks
        extra_state = step_all_robots(target_dt)

        step_counter += 1

        # Stream state to WebSocket clients at reduced rate
        if step_counter >= steps_per_stream:
            step_counter = 0
            state = _build_state_snapshot(world_id, world, engine)
            # Merge in drone/mobile robot state
            state["drones"] = extra_state.get("drones", {})
            state["mobile_robots"] = extra_state.get("mobile_robots", {})
            state["tasks"] = extra_state.get("tasks", {})
            await _broadcast(world_id, state)

        # Yield to event loop
        await asyncio.sleep(target_dt * world.physics.real_time_factor)


def _build_state_snapshot(world_id: str, world: World, engine) -> dict:
    """Build a JSON-serializable state snapshot for streaming."""
    robots_state = {}

    if _USE_PYBULLET:
        for body_id, info in engine._robot_bodies.items():
            joint_states = engine.get_joint_states(body_id)
            link_poses = engine.get_all_link_poses(body_id)
            robots_state[info["robot_id"]] = {
                "joints": {
                    name: {"position": s.position, "velocity": s.velocity}
                    for name, s in joint_states.items()
                },
                "links": {
                    name: {"position": pose[0], "orientation": pose[1]}
                    for name, pose in link_poses.items()
                },
            }
    else:
        # Simple engine: iterate loaded robots
        for body_id in list(engine._robots.keys()):
            joint_states = engine.get_joint_states(body_id)
            link_poses = engine.get_all_link_poses(body_id)
            robot_id = engine._robots[body_id].robot_id
            robots_state[robot_id] = {
                "joints": {
                    name: {"position": s.position, "velocity": s.velocity}
                    for name, s in joint_states.items()
                },
                "links": {
                    name: {"position": pose[0], "orientation": pose[1]}
                    for name, pose in link_poses.items()
                },
            }

    return {
        "type": "state_update",
        "sim_time": world.state.sim_time,
        "step_count": world.state.step_count,
        "robots": robots_state,
    }


async def _broadcast(world_id: str, message: dict):
    """Broadcast state to all connected WebSocket clients."""
    connections = _ws_connections.get(world_id, [])
    dead = []
    msg_str = json.dumps(message)

    for ws in connections:
        try:
            await ws.send_text(msg_str)
        except Exception:
            dead.append(ws)

    for ws in dead:
        connections.remove(ws)
