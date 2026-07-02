"""ROS Bridge Interface for RoboSim.

Provides a rosbridge-compatible WebSocket protocol so users can connect
their ROS2 nodes directly to our simulation. Follows the rosbridge v2.0 protocol.

Usage:
    1. Start RoboSim server (python server.py)
    2. Connect your ROS2 nodes via: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
       OR connect directly to ws://localhost:9090 from roslibjs
    3. Topics available:
       - /joint_states (sensor_msgs/JointState) [published by sim]
       - /cmd_joint_positions (std_msgs/Float64MultiArray) [subscribe to control]
       - /tf (tf2_msgs/TFMessage) [published by sim]
       - /robot_description (std_msgs/String) [URDF string]
       - /sim/status (std_msgs/String) [simulation state]

Protocol: rosbridge v2.0 (JSON over WebSocket)
Ref: https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md
"""

from __future__ import annotations
import asyncio
import json
import time
from typing import Optional
from fastapi import WebSocket, WebSocketDisconnect

# ROS message type constants
JOINT_STATE_TYPE = "sensor_msgs/JointState"
TF_MESSAGE_TYPE = "tf2_msgs/TFMessage"
FLOAT64_MULTI_ARRAY_TYPE = "std_msgs/Float64MultiArray"
STRING_TYPE = "std_msgs/String"


class ROSBridgeServer:
    """Rosbridge-compatible WebSocket server.

    Implements the rosbridge v2.0 protocol for publishing topics,
    subscribing to commands, and advertising services.
    """

    def __init__(self):
        self._subscribers: dict[str, list[WebSocket]] = {}  # topic -> [ws clients]
        self._publishers: dict[str, str] = {}  # topic -> message type
        self._connected_clients: list[WebSocket] = []
        self._joint_names: list[str] = []
        self._last_joint_states: dict[str, float] = {}
        self._command_callback = None

    def set_joint_names(self, names: list[str]) -> None:
        """Set the joint names for this robot."""
        self._joint_names = names

    def set_command_callback(self, callback) -> None:
        """Set callback for when joint commands arrive from ROS."""
        self._command_callback = callback

    async def handle_connection(self, websocket: WebSocket) -> None:
        """Handle a rosbridge WebSocket connection."""
        await websocket.accept()
        self._connected_clients.append(websocket)

        try:
            while True:
                data = await websocket.receive_text()
                msg = json.loads(data)
                await self._handle_message(websocket, msg)
        except WebSocketDisconnect:
            self._connected_clients.remove(websocket)
            # Remove from all subscriptions
            for topic_subs in self._subscribers.values():
                if websocket in topic_subs:
                    topic_subs.remove(websocket)

    async def _handle_message(self, ws: WebSocket, msg: dict) -> None:
        """Process an incoming rosbridge protocol message."""
        op = msg.get("op", "")

        if op == "subscribe":
            # Client wants to receive messages on a topic
            topic = msg.get("topic", "")
            if topic not in self._subscribers:
                self._subscribers[topic] = []
            if ws not in self._subscribers[topic]:
                self._subscribers[topic].append(ws)

        elif op == "unsubscribe":
            topic = msg.get("topic", "")
            if topic in self._subscribers and ws in self._subscribers[topic]:
                self._subscribers[topic].remove(ws)

        elif op == "publish":
            # Client is publishing a message (e.g., joint commands)
            topic = msg.get("topic", "")
            message = msg.get("msg", {})
            await self._handle_incoming_publish(topic, message)

        elif op == "advertise":
            # Client advertising they will publish on a topic
            topic = msg.get("topic", "")
            msg_type = msg.get("type", "")
            self._publishers[topic] = msg_type

        elif op == "call_service":
            # Service call (e.g., get robot description)
            service = msg.get("service", "")
            await self._handle_service_call(ws, msg)

    async def _handle_incoming_publish(self, topic: str, message: dict) -> None:
        """Handle messages published by ROS clients (commands)."""
        if topic in ("/cmd_joint_positions", "/joint_commands"):
            # Extract joint positions from message
            data = message.get("data", [])
            if data and self._joint_names:
                positions = {}
                for i, name in enumerate(self._joint_names):
                    if i < len(data):
                        positions[name] = data[i]
                if self._command_callback:
                    self._command_callback(positions)

        elif topic == "/cmd_vel":
            # Twist message for mobile robots
            linear = message.get("linear", {})
            angular = message.get("angular", {})
            # Could route to mobile robot controller

    async def _handle_service_call(self, ws: WebSocket, msg: dict) -> None:
        """Handle ROS service calls."""
        service = msg.get("service", "")
        call_id = msg.get("id", "")

        if service == "/get_robot_description":
            # Return URDF string
            response = {
                "op": "service_response",
                "id": call_id,
                "service": service,
                "values": {"description": ""},  # Would include actual URDF
                "result": True,
            }
            await ws.send_text(json.dumps(response))

    async def publish_joint_states(self, positions: dict[str, float], velocities: dict[str, float] = None) -> None:
        """Publish joint states to all subscribed clients."""
        self._last_joint_states = positions

        if "/joint_states" not in self._subscribers:
            return

        names = list(positions.keys())
        pos_values = [positions[n] for n in names]
        vel_values = [velocities.get(n, 0.0) for n in names] if velocities else [0.0] * len(names)

        msg = {
            "op": "publish",
            "topic": "/joint_states",
            "msg": {
                "header": {
                    "stamp": {"sec": int(time.time()), "nanosec": 0},
                    "frame_id": "base_link",
                },
                "name": names,
                "position": pos_values,
                "velocity": vel_values,
                "effort": [0.0] * len(names),
            },
        }

        await self._broadcast("/joint_states", msg)

    async def publish_tf(self, transforms: dict[str, tuple]) -> None:
        """Publish transform tree to subscribed clients."""
        if "/tf" not in self._subscribers:
            return

        tf_messages = []
        for link_name, (position, orientation) in transforms.items():
            tf_messages.append({
                "header": {
                    "stamp": {"sec": int(time.time()), "nanosec": 0},
                    "frame_id": "world",
                },
                "child_frame_id": link_name,
                "transform": {
                    "translation": {"x": position[0], "y": position[1], "z": position[2]},
                    "rotation": {"x": orientation[0], "y": orientation[1], "z": orientation[2], "w": orientation[3]},
                },
            })

        msg = {
            "op": "publish",
            "topic": "/tf",
            "msg": {"transforms": tf_messages},
        }

        await self._broadcast("/tf", msg)

    async def publish_sim_status(self, status: str, sim_time: float) -> None:
        """Publish simulation status."""
        if "/sim/status" not in self._subscribers:
            return

        msg = {
            "op": "publish",
            "topic": "/sim/status",
            "msg": {"data": json.dumps({"status": status, "sim_time": sim_time})},
        }
        await self._broadcast("/sim/status", msg)

    async def _broadcast(self, topic: str, msg: dict) -> None:
        """Send message to all clients subscribed to a topic."""
        subscribers = self._subscribers.get(topic, [])
        dead = []
        msg_str = json.dumps(msg)

        for ws in subscribers:
            try:
                await ws.send_text(msg_str)
            except Exception:
                dead.append(ws)

        for ws in dead:
            subscribers.remove(ws)

    @property
    def connected_count(self) -> int:
        return len(self._connected_clients)


# Global ROS bridge instance
ros_bridge = ROSBridgeServer()
