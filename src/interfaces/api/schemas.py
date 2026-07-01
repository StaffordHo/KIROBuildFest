"""Pydantic schemas for API request/response validation."""

from pydantic import BaseModel, Field
from typing import Optional


class WorldCreateRequest(BaseModel):
    """Request to create a new simulation world."""
    name: str = Field(default="default", description="World name")
    gravity_z: float = Field(default=-9.81, description="Gravity Z component (m/s²)")
    time_step: float = Field(default=1/240, description="Physics time step (seconds)")


class RobotCreateRequest(BaseModel):
    """Request to create a simple robot from parameters."""
    name: str = Field(description="Robot name")
    dof: int = Field(default=6, ge=1, le=12, description="Degrees of freedom")
    link_length: float = Field(default=0.1, ge=0.01, le=2.0, description="Link length (meters)")


class JointCommandRequest(BaseModel):
    """Request to set joint positions."""
    robot_id: str = Field(description="Robot ID or name")
    positions: dict[str, float] = Field(description="Map of joint_name -> target_position (rad)")


class SimControlRequest(BaseModel):
    """Request to control simulation lifecycle."""
    action: str = Field(description="One of: start, pause, stop, reset")


class WorldStateResponse(BaseModel):
    """Response containing world state snapshot."""
    world_id: str
    name: str
    status: str
    sim_time: float
    step_count: int
    robots: dict
