# RoboSim — Open Robotics Simulation Platform

A free, browser-based robotics simulation environment for designing, testing, and validating robot control systems without physical hardware.

## Value Proposition

1. **Hardware Stub** — Simulate robots you don't physically have. Test control algorithms against accurate kinematic/dynamic models.
2. **Model Porting** — Import open-source URDF/SDF robot descriptions. Validate configurations before deployment.
3. **Validation** — Run control pipelines against physics simulation. Catch errors before they damage real hardware.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Frontend (Three.js)                        │
│  3D Visualization │ Robot Config │ Control Panel │ Telemetry │
└─────────────────────────────┬───────────────────────────────┘
                              │ WebSocket + REST
┌─────────────────────────────┴───────────────────────────────┐
│                    Application Layer                          │
│  SimulationOrchestrator │ RobotManager │ ScenarioRunner      │
├──────────────────────────────────────────────────────────────┤
│                    Domain Layer                               │
│  Robot │ Joint │ Link │ Sensor │ Actuator │ World │ Scene    │
├──────────────────────────────────────────────────────────────┤
│                    Infrastructure Layer                       │
│  PhysicsEngine │ URDFParser │ AWS(Bedrock/S3/DynamoDB)       │
└──────────────────────────────────────────────────────────────┘
```

## Domain Ontology

```
World
├── Scene
│   ├── Robot (aggregate root)
│   │   ├── Link (rigid body)
│   │   │   ├── Visual (mesh/geometry)
│   │   │   ├── Collision (simplified geometry)
│   │   │   └── Inertial (mass properties)
│   │   ├── Joint (connection between links)
│   │   │   ├── JointType (revolute|prismatic|fixed|continuous|floating)
│   │   │   ├── JointLimits (position, velocity, effort)
│   │   │   └── JointState (position, velocity, effort)
│   │   ├── Sensor
│   │   │   ├── ForceTorqueSensor
│   │   │   ├── IMU
│   │   │   └── Camera
│   │   └── Actuator
│   │       ├── PositionController
│   │       ├── VelocityController
│   │       └── EffortController
│   ├── StaticObject (obstacles, surfaces)
│   └── DynamicObject (manipulable objects)
├── PhysicsConfig
│   ├── Gravity
│   ├── TimeStep
│   └── SolverIterations
└── SimulationState
    ├── Clock
    ├── Running/Paused/Stopped
    └── Telemetry
```

## Tech Stack

- **Backend:** Python 3.11+ / FastAPI / WebSocket
- **Physics:** PyBullet (Bullet Physics SDK)
- **Robot Description:** URDF parser (urdfpy)
- **Frontend:** Three.js + TypeScript
- **AWS:** Bedrock (NL robot programming), S3 (model storage), DynamoDB (sessions)
- **IaC:** AWS CDK (Python)

## Quick Start

```bash
# Backend
cd src
pip install -r requirements.txt
python -m interfaces.api.main

# Frontend
cd frontend
npm install
npm run dev
```

## Project Structure (Clean Architecture)

```
src/
├── domain/           # Enterprise business rules (no dependencies)
│   ├── models/       # Entities, Value Objects, Aggregates
│   └── services/     # Domain services
├── application/      # Application business rules
│   ├── commands/     # Write operations (CQRS)
│   └── queries/      # Read operations (CQRS)
├── infrastructure/   # Frameworks & drivers
│   ├── physics/      # PyBullet integration
│   ├── persistence/  # DynamoDB, file storage
│   └── aws/          # Bedrock, S3 clients
└── interfaces/       # Controllers & presenters
    ├── api/          # REST endpoints
    └── websocket/    # Real-time sim streaming
```

## License

MIT
