"""Integration tests for the FastAPI REST endpoints."""
import pytest
import sys
sys.path.insert(0, ".")

from fastapi.testclient import TestClient
from src.interfaces.api.main import app


client = TestClient(app)


class TestHealthEndpoint:
    def test_health(self):
        r = client.get("/health")
        assert r.status_code == 200
        assert r.json()["status"] == "ok"


class TestWorldEndpoints:
    def test_create_world(self):
        r = client.post("/worlds", json={"name": "test"})
        assert r.status_code == 201
        data = r.json()
        assert "world_id" in data
        assert data["status"] == "idle"

    def test_list_worlds(self):
        client.post("/worlds", json={"name": "list_test"})
        r = client.get("/worlds")
        assert r.status_code == 200
        assert len(r.json()) > 0

    def test_get_world_not_found(self):
        r = client.get("/worlds/nonexistent")
        assert r.status_code == 404


class TestRobotEndpoints:
    def _create_world(self):
        r = client.post("/worlds", json={"name": "robot_test"})
        return r.json()["world_id"]

    def test_create_robot(self):
        wid = self._create_world()
        r = client.post(f"/worlds/{wid}/robots",
                        json={"name": "arm", "dof": 6, "link_length": 0.1})
        assert r.status_code == 200
        data = r.json()
        assert data["dof"] == 6
        assert "joint_1" in data["joints"]

    def test_set_joint_commands(self):
        wid = self._create_world()
        r = client.post(f"/worlds/{wid}/robots",
                        json={"name": "arm", "dof": 3})
        rid = r.json()["robot_id"]

        r = client.post(f"/worlds/{wid}/joints",
                        json={"robot_id": rid, "positions": {"joint_1": 0.5}})
        assert r.status_code == 200
        assert r.json()["applied"]["joint_1"] == 0.5

    def test_simulation_control(self):
        wid = self._create_world()
        r = client.post(f"/worlds/{wid}/control", json={"action": "start"})
        assert r.status_code == 200
        assert r.json()["status"] == "running"

        r = client.post(f"/worlds/{wid}/control", json={"action": "pause"})
        assert r.json()["status"] == "paused"

    def test_get_robot_geometry(self):
        wid = self._create_world()
        r = client.post(f"/worlds/{wid}/robots",
                        json={"name": "arm", "dof": 4})
        rid = r.json()["robot_id"]

        r = client.get(f"/worlds/{wid}/robots/{rid}/geometry")
        assert r.status_code == 200
        geo = r.json()
        assert "joints" in geo
        assert "link_bodies" in geo


class TestDroneEndpoints:
    def test_create_drone(self):
        r = client.post("/api/drones",
                        json={"name": "quad", "drone_type": "quadcopter"})
        assert r.status_code == 201
        data = r.json()
        assert data["num_rotors"] == 4
        assert data["armed"] is True

    def test_command_drone(self):
        r = client.post("/api/drones",
                        json={"name": "cmd_test", "drone_type": "quadcopter"})
        drone_id = r.json()["drone_id"]

        r = client.post("/api/drones/command",
                        json={"drone_id": drone_id, "thrust": 0.6, "roll": 0.1,
                              "pitch": -0.1, "yaw_rate": 0.2})
        assert r.status_code == 200
        assert r.json()["command"]["thrust"] == 0.6

    def test_get_drone_state(self):
        r = client.post("/api/drones",
                        json={"name": "state_test", "drone_type": "quadcopter"})
        drone_id = r.json()["drone_id"]

        r = client.get(f"/api/drones/{drone_id}/state")
        assert r.status_code == 200
        state = r.json()
        assert "position" in state
        assert "battery" in state


class TestMobileRobotEndpoints:
    def test_create_mobile_robot(self):
        r = client.post("/api/mobile-robots",
                        json={"name": "bot", "drive_type": "differential"})
        assert r.status_code == 201
        assert r.json()["drive_type"] == "differential"

    def test_command_mobile_robot(self):
        r = client.post("/api/mobile-robots",
                        json={"name": "cmd_bot", "drive_type": "differential"})
        rid = r.json()["robot_id"]

        r = client.post("/api/mobile-robots/command",
                        json={"robot_id": rid, "linear_x": 0.5, "angular_z": 0.2})
        assert r.status_code == 200


class TestModelLibrary:
    def test_get_catalog(self):
        r = client.get("/api/models/catalog")
        assert r.status_code == 200
        data = r.json()
        assert data["total"] > 0
        assert len(data["models"]) > 0

    def test_get_model_details(self):
        r = client.get("/api/models/catalog/kuka_iiwa")
        assert r.status_code == 200
        assert r.json()["name"] == "KUKA iiwa 14"

    def test_model_not_found(self):
        r = client.get("/api/models/catalog/nonexistent")
        assert r.status_code == 404

    def test_get_categories(self):
        r = client.get("/api/models/categories")
        assert r.status_code == 200
        cats = r.json()["categories"]
        assert len(cats) > 0


class TestManipulationTasks:
    def test_create_pick_and_place(self):
        r = client.post("/api/tasks", json={"scenario": "pick_and_place"})
        assert r.status_code == 201
        data = r.json()
        assert data["num_objects"] == 3
        assert data["num_goals"] == 2

    def test_create_sorting(self):
        r = client.post("/api/tasks", json={"scenario": "sorting"})
        assert r.status_code == 201
        assert r.json()["num_objects"] == 5

    def test_invalid_scenario(self):
        r = client.post("/api/tasks", json={"scenario": "nonexistent"})
        assert r.status_code == 400


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
