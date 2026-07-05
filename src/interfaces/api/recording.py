"""Trajectory Recording & Export for RoboSim.

Records joint states over time and exports as JSON/CSV for analysis.
Scientists and students can download simulation data for:
- Post-processing in MATLAB/Python
- Training ML models
- Validating control algorithms
- Publishing reproducible results
"""

from __future__ import annotations
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from typing import Optional
import time
import json

router = APIRouter(prefix="/recording", tags=["Recording & Export"])

# Active recordings: world_id -> RecordingSession
_recordings: dict[str, "RecordingSession"] = {}


class RecordingSession:
    """Stores timestamped joint state data."""

    def __init__(self, world_id: str, robot_id: str):
        self.world_id = world_id
        self.robot_id = robot_id
        self.start_time = time.time()
        self.frames: list[dict] = []
        self.is_recording = True

    def add_frame(self, sim_time: float, joint_states: dict, link_poses: dict = None):
        """Record a single timestep."""
        if not self.is_recording:
            return
        frame = {
            "sim_time": sim_time,
            "wall_time": time.time() - self.start_time,
            "joints": joint_states,
        }
        if link_poses:
            frame["link_poses"] = link_poses
        self.frames.append(frame)

    def stop(self):
        self.is_recording = False

    @property
    def duration(self) -> float:
        if not self.frames:
            return 0.0
        return self.frames[-1]["sim_time"] - self.frames[0]["sim_time"]

    @property
    def num_frames(self) -> int:
        return len(self.frames)

    def to_json(self) -> dict:
        """Export as structured JSON."""
        return {
            "metadata": {
                "world_id": self.world_id,
                "robot_id": self.robot_id,
                "duration_seconds": self.duration,
                "num_frames": self.num_frames,
                "start_time_unix": self.start_time,
                "export_format": "robosim_trajectory_v1",
            },
            "frames": self.frames,
        }

    def to_csv_lines(self) -> list[str]:
        """Export as CSV lines (header + data)."""
        if not self.frames:
            return ["sim_time,wall_time"]

        # Get all joint names from first frame
        joint_names = list(self.frames[0].get("joints", {}).keys())
        header = "sim_time,wall_time," + ",".join(f"{j}_pos,{j}_vel" for j in joint_names)
        lines = [header]

        for frame in self.frames:
            joints = frame.get("joints", {})
            values = [f"{frame['sim_time']:.6f}", f"{frame['wall_time']:.6f}"]
            for j in joint_names:
                jstate = joints.get(j, {})
                values.append(f"{jstate.get('position', 0.0):.6f}")
                values.append(f"{jstate.get('velocity', 0.0):.6f}")
            lines.append(",".join(values))

        return lines


class StartRecordingRequest(BaseModel):
    world_id: str
    robot_id: str


class StopRecordingRequest(BaseModel):
    world_id: str


@router.post("/start")
async def start_recording(request: StartRecordingRequest):
    """Start recording joint states for a world."""
    session = RecordingSession(request.world_id, request.robot_id)
    _recordings[request.world_id] = session
    return {
        "status": "recording",
        "world_id": request.world_id,
        "robot_id": request.robot_id,
    }


@router.post("/stop")
async def stop_recording(request: StopRecordingRequest):
    """Stop recording and return summary."""
    session = _recordings.get(request.world_id)
    if not session:
        raise HTTPException(404, "No active recording for this world")

    session.stop()
    return {
        "status": "stopped",
        "duration": session.duration,
        "num_frames": session.num_frames,
        "world_id": request.world_id,
    }


@router.get("/export/{world_id}/json")
async def export_json(world_id: str):
    """Export recorded trajectory as JSON."""
    session = _recordings.get(world_id)
    if not session:
        raise HTTPException(404, "No recording found for this world")

    return JSONResponse(
        content=session.to_json(),
        headers={"Content-Disposition": f"attachment; filename=trajectory_{world_id[:8]}.json"},
    )


@router.get("/export/{world_id}/csv")
async def export_csv(world_id: str):
    """Export recorded trajectory as CSV."""
    session = _recordings.get(world_id)
    if not session:
        raise HTTPException(404, "No recording found for this world")

    csv_content = "\n".join(session.to_csv_lines())
    return JSONResponse(
        content={"csv": csv_content, "num_frames": session.num_frames},
        headers={"Content-Disposition": f"attachment; filename=trajectory_{world_id[:8]}.csv"},
    )


@router.get("/export/{world_id}/ros")
async def export_ros_trajectory(world_id: str):
    """Export as ROS2 JointTrajectory message format.

    Output can be published directly to a /joint_trajectory_controller/joint_trajectory
    topic for real robot execution via:
        ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory <data>
    """
    session = _recordings.get(world_id)
    if not session:
        raise HTTPException(404, "No recording found for this world")

    if not session.frames:
        raise HTTPException(400, "Recording has no frames")

    # Build ROS2 JointTrajectory message structure
    joint_names = list(session.frames[0].get("joints", {}).keys())
    points = []

    for frame in session.frames:
        joints = frame.get("joints", {})
        point = {
            "positions": [joints.get(j, {}).get("position", 0.0) for j in joint_names],
            "velocities": [joints.get(j, {}).get("velocity", 0.0) for j in joint_names],
            "accelerations": [],  # Not tracked
            "effort": [],
            "time_from_start": {
                "sec": int(frame["sim_time"]),
                "nanosec": int((frame["sim_time"] % 1) * 1e9),
            },
        }
        points.append(point)

    ros_msg = {
        "header": {
            "stamp": {"sec": 0, "nanosec": 0},
            "frame_id": "base_link",
        },
        "joint_names": joint_names,
        "points": points,
    }

    return JSONResponse(
        content={
            "format": "trajectory_msgs/JointTrajectory",
            "description": "ROS2-compatible JointTrajectory. Publish to /joint_trajectory_controller/joint_trajectory",
            "usage": "ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '<paste_message>'",
            "message": ros_msg,
        },
        headers={"Content-Disposition": f"attachment; filename=ros_trajectory_{world_id[:8]}.json"},
    )


@router.get("/export/{world_id}/ml")
async def export_ml_dataset(world_id: str):
    """Export as ML-ready dataset format (compatible with robomimic/D4RL).

    Structure follows the observation-action format used in:
    - robomimic (for imitation learning)
    - D4RL (for offline RL)
    - LeRobot (Hugging Face robotics dataset format)

    Each timestep contains:
    - observations: joint positions, velocities
    - actions: target positions (what was commanded)
    - rewards: placeholder (0.0, can be labeled post-hoc)
    - dones: whether episode ended
    """
    session = _recordings.get(world_id)
    if not session:
        raise HTTPException(404, "No recording found for this world")

    if not session.frames:
        raise HTTPException(400, "Recording has no frames")

    joint_names = list(session.frames[0].get("joints", {}).keys())
    num_joints = len(joint_names)

    observations = []
    actions = []
    timestamps = []

    for i, frame in enumerate(session.frames):
        joints = frame.get("joints", {})

        # Observation: [pos_1, pos_2, ..., vel_1, vel_2, ...]
        positions = [joints.get(j, {}).get("position", 0.0) for j in joint_names]
        velocities = [joints.get(j, {}).get("velocity", 0.0) for j in joint_names]
        obs = positions + velocities
        observations.append(obs)

        # Action: next target position (use next frame's position, or same for last)
        if i < len(session.frames) - 1:
            next_joints = session.frames[i + 1].get("joints", {})
            action = [next_joints.get(j, {}).get("position", 0.0) for j in joint_names]
        else:
            action = positions  # Last frame: action = stay
        actions.append(action)
        timestamps.append(frame["sim_time"])

    dataset = {
        "format": "robosim_ml_dataset_v1",
        "description": "ML-ready trajectory dataset. Compatible with robomimic/D4RL/LeRobot formats.",
        "metadata": {
            "robot_id": session.robot_id,
            "num_episodes": 1,
            "num_timesteps": len(observations),
            "observation_dim": num_joints * 2,  # pos + vel
            "action_dim": num_joints,
            "joint_names": joint_names,
            "dt": session.duration / max(len(session.frames) - 1, 1),
        },
        "episodes": [
            {
                "episode_id": 0,
                "num_steps": len(observations),
                "observations": observations,
                "actions": actions,
                "rewards": [0.0] * len(observations),  # Unlabeled
                "dones": [False] * (len(observations) - 1) + [True],
                "timestamps": timestamps,
            }
        ],
        "usage": {
            "python": "import json; data = json.load(open('dataset.json')); obs = np.array(data['episodes'][0]['observations'])",
            "robomimic": "Convert observations/actions to HDF5 with robomimic.utils.dataset_utils",
            "pytorch": "dataset = torch.tensor(data['episodes'][0]['observations'])",
        },
    }

    return JSONResponse(
        content=dataset,
        headers={"Content-Disposition": f"attachment; filename=ml_dataset_{world_id[:8]}.json"},
    )


@router.get("/status/{world_id}")
async def recording_status(world_id: str):
    """Check recording status."""
    session = _recordings.get(world_id)
    if not session:
        return {"status": "idle", "recording": False}
    return {
        "status": "recording" if session.is_recording else "stopped",
        "recording": session.is_recording,
        "num_frames": session.num_frames,
        "duration": session.duration,
    }


def record_frame(world_id: str, sim_time: float, joint_states: dict, link_poses: dict = None):
    """Called from simulation loop to record a frame (if recording is active)."""
    session = _recordings.get(world_id)
    if session and session.is_recording:
        session.add_frame(sim_time, joint_states, link_poses)
