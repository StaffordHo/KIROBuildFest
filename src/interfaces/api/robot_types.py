"""API endpoints for different robot types: drones, mobile robots, manipulation tasks."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional
import uuid

router = APIRouter()

# --- Schemas ---

class DroneCreateRequest(BaseModel):
    name: str = Field(default="quadcopter")
    drone_type: str = Field(default="quadcopter", description="quadcopter, hexacopter, octocopter")
    mass: float = Field(default=1.5, ge=0.1, le=50.0)
    arm_length: float = Field(default=0.25, ge=0.05, le=2.0)


class DroneCommandRequest(BaseModel):
    drone_id: str
    thrust: float = Field(default=0.0, ge=0.0, le=1.0)
    roll: float = Field(default=0.0, ge=-1.0, le=1.0)
    pitch: float = Field(default=0.0, ge=-1.0, le=1.0)
    yaw_rate: float = Field(default=0.0, ge=-1.0, le=1.0)


class MobileRobotCreateRequest(BaseModel):
    name: str = Field(default="turtlebot")
    drive_type: str = Field(default="differential", description="differential, ackermann, omnidirectional")
    mass: float = Field(default=10.0)
    wheel_radius: float = Field(default=0.05)
    body_length: float = Field(default=0.4)


class MobileRobotCommandRequest(BaseModel):
    robot_id: str
    linear_x: float = Field(default=0.0, ge=-2.0, le=2.0)
    linear_y: float = Field(default=0.0, ge=-2.0, le=2.0)
    angular_z: float = Field(default=0.0, ge=-3.0, le=3.0)


class ManipulationTaskRequest(BaseModel):
    scenario: str = Field(default="pick_and_place", description="pick_and_place, sorting")


# These will be populated by the main app
_drones = {}
_mobile_robots = {}
_manipulation_tasks = {}
_drone_physics = None


def init_robot_types():
    """Initialize physics engines for robot types."""
    global _drone_physics
    from ...infrastructure.physics.drone_physics import DronePhysics
    _drone_physics = DronePhysics()


# --- Drone Endpoints ---

@router.post("/drones", status_code=201)
async def create_drone(request: DroneCreateRequest):
    """Create a drone in the simulation."""
    from ...domain.models.drone import Drone, DroneId, DroneConfig, DroneType

    type_map = {
        "quadcopter": DroneType.QUADCOPTER,
        "hexacopter": DroneType.HEXACOPTER,
        "octocopter": DroneType.OCTOCOPTER,
    }
    drone_type = type_map.get(request.drone_type, DroneType.QUADCOPTER)

    config = DroneConfig(
        drone_type=drone_type,
        mass=request.mass,
        arm_length=request.arm_length,
    )

    drone = Drone(
        id=DroneId(str(uuid.uuid4())),
        name=request.name,
        config=config,
    )
    drone.arm()

    _drones[str(drone.id)] = drone

    return {
        "drone_id": str(drone.id),
        "name": drone.name,
        "type": drone.config.drone_type.value,
        "num_rotors": len(drone.config.rotors),
        "mass": drone.config.mass,
        "armed": drone.state.armed,
    }


@router.get("/drones")
async def list_drones():
    """List all active drones."""
    return [
        {
            "drone_id": did,
            "name": d.name,
            "type": d.config.drone_type.value,
            "altitude": d.state.altitude,
            "battery": d.state.battery_level,
            "armed": d.state.armed,
        }
        for did, d in _drones.items()
    ]


@router.post("/drones/command")
async def command_drone(request: DroneCommandRequest):
    """Send flight command to a drone."""
    drone = _drones.get(request.drone_id)
    if not drone:
        raise HTTPException(404, "Drone not found")

    drone.set_command(request.thrust, request.roll, request.pitch, request.yaw_rate)

    return {
        "drone_id": request.drone_id,
        "command": {
            "thrust": drone.command.thrust,
            "roll": drone.command.roll,
            "pitch": drone.command.pitch,
            "yaw_rate": drone.command.yaw_rate,
        },
    }


@router.get("/drones/{drone_id}/state")
async def get_drone_state(drone_id: str):
    """Get detailed drone state."""
    drone = _drones.get(drone_id)
    if not drone:
        raise HTTPException(404, "Drone not found")

    return {
        "drone_id": drone_id,
        "name": drone.name,
        "position": {
            "x": drone.state.pose.position.x,
            "y": drone.state.pose.position.y,
            "z": drone.state.pose.position.z,
        },
        "velocity": {
            "x": drone.state.linear_velocity.x,
            "y": drone.state.linear_velocity.y,
            "z": drone.state.linear_velocity.z,
        },
        "attitude": {
            "roll": getattr(drone.state, '_roll', 0.0),
            "pitch": getattr(drone.state, '_pitch', 0.0),
            "yaw": getattr(drone.state, '_yaw', 0.0),
        },
        "rotor_speeds": drone.state.rotor_speeds,
        "battery": drone.state.battery_level,
        "armed": drone.state.armed,
        "airborne": drone.state.is_airborne,
    }


# --- Mobile Robot Endpoints ---

@router.post("/mobile-robots", status_code=201)
async def create_mobile_robot(request: MobileRobotCreateRequest):
    """Create a mobile robot."""
    from ...domain.models.mobile_robot import MobileRobot, MobileRobotId, MobileRobotConfig, DriveType

    type_map = {
        "differential": DriveType.DIFFERENTIAL,
        "ackermann": DriveType.ACKERMANN,
        "omnidirectional": DriveType.OMNI,
        "skid_steer": DriveType.SKID_STEER,
    }
    drive_type = type_map.get(request.drive_type, DriveType.DIFFERENTIAL)

    config = MobileRobotConfig(
        drive_type=drive_type,
        mass=request.mass,
        wheel_radius=request.wheel_radius,
        body_length=request.body_length,
    )

    robot = MobileRobot(
        id=MobileRobotId(str(uuid.uuid4())),
        name=request.name,
        config=config,
    )

    _mobile_robots[str(robot.id)] = robot

    return {
        "robot_id": str(robot.id),
        "name": robot.name,
        "drive_type": robot.config.drive_type.value,
        "mass": robot.config.mass,
    }


@router.post("/mobile-robots/command")
async def command_mobile_robot(request: MobileRobotCommandRequest):
    """Send velocity command to a mobile robot."""
    robot = _mobile_robots.get(request.robot_id)
    if not robot:
        raise HTTPException(404, "Mobile robot not found")

    robot.set_velocity(request.linear_x, request.angular_z, request.linear_y)

    return {
        "robot_id": request.robot_id,
        "command": {
            "linear_x": robot.command.linear_x,
            "linear_y": robot.command.linear_y,
            "angular_z": robot.command.angular_z,
        },
    }


@router.get("/mobile-robots/{robot_id}/state")
async def get_mobile_robot_state(robot_id: str):
    """Get mobile robot state."""
    robot = _mobile_robots.get(robot_id)
    if not robot:
        raise HTTPException(404, "Mobile robot not found")
    return {"robot_id": robot_id, **robot.get_state_dict()}


# --- Manipulation Task Endpoints ---

@router.post("/tasks", status_code=201)
async def create_manipulation_task(request: ManipulationTaskRequest):
    """Create a manipulation task scenario."""
    from ...domain.models.manipulation import (
        create_pick_and_place_scenario,
        create_sorting_scenario,
    )

    if request.scenario == "pick_and_place":
        task = create_pick_and_place_scenario()
    elif request.scenario == "sorting":
        task = create_sorting_scenario()
    else:
        raise HTTPException(400, f"Unknown scenario: {request.scenario}")

    _manipulation_tasks[str(task.id)] = task
    task.status = task.status.IN_PROGRESS

    return {
        "task_id": str(task.id),
        "name": task.name,
        "description": task.description,
        "num_goals": len(task.goals),
        "num_objects": len(task.objects),
        "time_limit": task.time_limit,
        "objects": [
            {"name": o.name, "shape": o.shape.value, "position": [o.pose.position.x, o.pose.position.y, o.pose.position.z]}
            for o in task.objects
        ],
        "zones": [
            {"name": z.name, "position": [z.pose.position.x, z.pose.position.y, z.pose.position.z]}
            for z in task.placement_zones
        ],
    }


@router.get("/tasks/{task_id}")
async def get_task_state(task_id: str):
    """Get manipulation task progress."""
    task = _manipulation_tasks.get(task_id)
    if not task:
        raise HTTPException(404, "Task not found")

    return {
        "task_id": task_id,
        "name": task.name,
        "status": task.status.value,
        "progress": task.progress,
        "completed_goals": task.completed_goals,
        "total_goals": len(task.goals),
        "elapsed_time": task.elapsed_time,
        "time_limit": task.time_limit,
    }


# --- Step all robot types (called from sim loop) ---

def step_all(dt: float) -> dict:
    """Step physics for all robot types. Returns state for streaming."""
    global _drone_physics

    if _drone_physics is None:
        init_robot_types()

    state = {"drones": {}, "mobile_robots": {}, "tasks": {}}

    # Step drones
    for did, drone in _drones.items():
        _drone_physics.step(drone, dt)
        state["drones"][did] = {
            "position": [drone.state.pose.position.x, drone.state.pose.position.y, drone.state.pose.position.z],
            "orientation": [
                drone.state.pose.orientation.x, drone.state.pose.orientation.y,
                drone.state.pose.orientation.z, drone.state.pose.orientation.w,
            ],
            "velocity": [drone.state.linear_velocity.x, drone.state.linear_velocity.y, drone.state.linear_velocity.z],
            "rotor_speeds": drone.state.rotor_speeds,
            "battery": drone.state.battery_level,
        }

    # Step mobile robots
    for rid, robot in _mobile_robots.items():
        robot.step(dt)
        state["mobile_robots"][rid] = robot.get_state_dict()

    # Update manipulation tasks
    for tid, task in _manipulation_tasks.items():
        task.update(dt)
        state["tasks"][tid] = {
            "status": task.status.value,
            "progress": task.progress,
            "elapsed_time": task.elapsed_time,
        }

    return state
