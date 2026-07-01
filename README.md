# RoboSim — Open Robotics Simulation Platform

**🌐 Live Demo: [https://kirobuildfest.onrender.com](https://kirobuildfest.onrender.com)**

A free, browser-based robotics simulation environment for designing, testing, and validating robot control systems without physical hardware. No installation required — just open the link and start simulating.

## Why RoboSim?

| | NVIDIA Omniverse | RoboSim |
|---|---|---|
| **Cost** | Requires RTX GPU + license | Free, runs in any browser |
| **Setup** | Heavy install (~30GB) | Zero install — open a URL |
| **Access** | Desktop only | Any device with a browser |
| **Robot Format** | USD (proprietary extensions) | URDF (industry standard) |
| **Target** | Enterprise with GPU clusters | Anyone — students to startups |

## Core Value Proposition

1. **Hardware Stub** — Simulate robots you don't physically have. Test control algorithms against accurate kinematic and dynamic models.
2. **Model Porting** — Import open-source URDF robot descriptions from the ROS ecosystem. One-click loading from curated library.
3. **Validation** — Run control pipelines against physics simulation. Catch errors before they damage real hardware worth thousands of dollars.

## Features

### Robot Types
- **Manipulator Arms** — 6/7/9-DOF serial chains (Franka Panda, KUKA iiwa)
- **Quadrupeds** — 12/16-DOF legged robots (Unitree A1, Laikago, Ghost Minitaur)
- **Mobile Robots** — Differential drive, Ackermann, omnidirectional (TurtleBot3, Jackal, MIT Racecar)
- **Drones** — Quadcopter/hexacopter with 6-DOF rigid body dynamics
- **Benchmarks** — Cart-pole and other classic control problems

### Model Library (10 verified robots)
| Robot | Category | DOF | Manufacturer |
|-------|----------|-----|--------------|
| Franka Emika Panda | Manipulator | 9 | Franka Emika |
| KUKA iiwa 14 | Collaborative | 7 | KUKA |
| TurtleBot3 Burger | Mobile | 2 | ROBOTIS |
| Clearpath Jackal | Mobile (UGV) | 4 | Clearpath Robotics |
| MIT Racecar | Mobile (Ackermann) | 6 | MIT |
| Unitree A1 | Quadruped | 12 | Unitree |
| Unitree Laikago | Quadruped | 12 | Unitree |
| Ghost Minitaur | Quadruped | 16 | Ghost Robotics |
| R2D2 | Articulated Mobile | 8 | PyBullet |
| Cart-Pole | RL Benchmark | 2 | Classic |

### Simulation Capabilities
- Real-time physics at 240Hz with WebSocket streaming at 60Hz
- Forward kinematics with proper cumulative rotation matrices
- PD joint control with configurable gains
- Drone flight dynamics (thrust, drag, PID attitude stabilization)
- Differential/Ackermann/omnidirectional drive kinematics
- Pick-and-place manipulation task scenarios
- URDF file upload for custom robot models

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Frontend (Three.js)                        │
│  3D Visualization │ Robot Config │ Control Panel │ Telemetry │
└─────────────────────────────┬───────────────────────────────┘
                              │ WebSocket + REST
┌─────────────────────────────┴───────────────────────────────┐
│                    Application Layer (FastAPI)                │
│  SimulationOrchestrator │ RobotManager │ ModelLibrary        │
├──────────────────────────────────────────────────────────────┤
│                    Domain Layer (DDD)                         │
│  Robot │ Joint │ Link │ Sensor │ Drone │ MobileRobot │ World │
├──────────────────────────────────────────────────────────────┤
│                    Infrastructure Layer                       │
│  PhysicsEngine │ URDFParser │ DronePhysics │ AWS (future)    │
└──────────────────────────────────────────────────────────────┘
```

## Tech Stack

- **Backend:** Python 3.11 / FastAPI / WebSocket
- **Physics:** Custom kinematic engine (PyBullet-ready for full dynamics)
- **Robot Description:** URDF parser with full joint/link/inertia support
- **Frontend:** Three.js + vanilla JavaScript (zero build step)
- **Deployment:** Render.com (free tier) / Docker
- **Future:** AWS Bedrock (NL robot programming), S3 (model storage), DynamoDB

## Quick Start (Local Development)

```bash
# Clone
git clone https://github.com/StaffordHo/KIROBuildFest.git
cd KIROBuildFest

# Install dependencies
pip install -r requirements.txt

# Run server
python server.py

# Open browser
# http://localhost:8000
```

## API Documentation

Interactive Swagger docs available at `/docs` when the server is running.

Key endpoints:
- `POST /worlds` — Create simulation world
- `POST /worlds/{id}/robots` — Add robot arm
- `POST /worlds/{id}/robots/upload` — Upload custom URDF
- `POST /worlds/{id}/control` — Start/pause/reset simulation
- `POST /worlds/{id}/joints` — Send joint commands
- `GET /api/models/catalog` — Browse model library
- `POST /api/models/fetch` — One-click load from library
- `POST /api/drones` — Create drone
- `POST /api/mobile-robots` — Create ground robot
- `WS /ws/{world_id}` — Real-time state streaming

## Project Structure (Clean Architecture)

```
src/
├── domain/           # Enterprise business rules (no dependencies)
│   ├── models/       # Robot, Joint, Link, Drone, MobileRobot, World
│   └── services/     # SimulationService, KinematicsService
├── infrastructure/   # Frameworks & drivers
│   ├── physics/      # SimpleEngine, PyBulletEngine, DronePhysics, URDFLoader
│   ├── persistence/  # (Future: DynamoDB)
│   └── aws/          # (Future: Bedrock, S3)
└── interfaces/       # Controllers & presenters
    ├── api/          # FastAPI REST + WebSocket + ModelLibrary
    └── websocket/    # Real-time streaming
```

## Domain Ontology

```
World
├── Scene
│   ├── Robot (aggregate root) — Serial manipulators
│   │   ├── Link (rigid body) → Visual, Collision, Inertial
│   │   ├── Joint → JointType, JointLimits, JointState
│   │   ├── Sensor → ForceTorque, IMU, Camera
│   │   └── Actuator → Position/Velocity/Effort control
│   ├── Drone (aggregate) — Multirotor UAVs
│   │   ├── RotorConfig → thrust, RPM, torque coefficient
│   │   ├── DroneState → pose, velocity, battery
│   │   └── DroneCommand → thrust, roll, pitch, yaw
│   ├── MobileRobot (aggregate) — Ground vehicles
│   │   ├── DriveType → differential, Ackermann, omni
│   │   └── Odometry → x, y, theta
│   └── ManipulationTask — Pick-and-place scenarios
│       ├── GraspableObject → shape, mass, friction
│       └── PlacementZone → target position, tolerance
├── PhysicsConfig → gravity, time_step, solver_iterations
└── SimulationState → clock, status, telemetry
```

## Roadmap

- [x] Core simulation engine with URDF support
- [x] Real-time 3D visualization (Three.js)
- [x] Model library with 10 verified robots
- [x] Drone flight simulation
- [x] Mobile robot kinematics
- [x] Manipulation task framework
- [x] Public deployment (Render.com)
- [ ] AWS Bedrock integration (natural language robot programming)
- [ ] Collision detection and contact dynamics
- [ ] Multi-user collaborative sessions
- [ ] ROS2 bridge for real robot integration
- [ ] GPU-accelerated physics via AWS

## Credits

**Built by [Stafford Ho Sheng Xian](https://github.com/StaffordHo)**

Built with [Kiro](https://kiro.dev) — the AI-powered development environment by AWS.

Special thanks to the **AWS team** for providing credits through the **Kiro BuildFest** hackathon, enabling rapid development and deployment of this platform.

## License

MIT
