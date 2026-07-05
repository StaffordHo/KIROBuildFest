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
