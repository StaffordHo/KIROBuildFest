## Inspiration

Robotics simulation today is gatekept by expensive tooling. NVIDIA Omniverse Isaac Sim demands an RTX GPU, 30GB of disk space, and a steep learning curve. Gazebo requires a full ROS installation on Linux. For the majority of robotics students, indie developers, and startups across Southeast Asia and beyond — this barrier is real.

I wanted to build **the Figma of robotics simulation** — open a URL, pick a robot, test your algorithm. Done. No install. No GPU. No license fee.

The question was simple: _Can a browser deliver enough physics fidelity to validate robot control before deployment to real hardware?_

## What I Built

**RoboSim** is a free, browser-based robotics simulation platform supporting manipulator arms, quadrupeds, mobile robots, and drones — all running real-time physics with standard URDF model support.

**Live demo:** [https://kirobuildfest.onrender.com](https://kirobuildfest.onrender.com)

### Core Capabilities

- **10 verified robot models** from the PyBullet/ROS ecosystem (Franka Panda, KUKA iiwa, Unitree A1, TurtleBot3, etc.)
- **Real-time kinematic simulation** at 240Hz physics / 60Hz rendering
- **Algorithm editor** — write and execute control code directly in the browser
- **Environment presets** — warehouse, tabletop, outdoor with simulated 360° LiDAR
- **One-click demo scenarios** — pick & place, warehouse sorting, quadruped gait, drone survey
- **AI assistant** for natural-language → control algorithm generation
- **URDF upload** — bring your own robot model
- **Mobile responsive** — works on phones and tablets

## How I Built It

### Architecture: Domain-Driven Design

I adopted **Clean Architecture** with a strong enterprise ontology, designed for future scalability:

```
Domain Layer (zero dependencies)
├── Robot, Joint, Link, Sensor, Actuator
├── Drone, MobileRobot
├── World, PhysicsConfig, SimulationState
└── ManipulationTask, GraspableObject

Infrastructure Layer (adapters)
├── SimplePhysicsEngine (pure Python FK)
├── PyBulletEngine (full dynamics, ready for production)
├── URDFLoader, DronePhysics
└── ROSBridgeServer (rosbridge v2.0 protocol)

Interface Layer (API + Frontend)
├── FastAPI REST + WebSocket
├── Three.js 3D visualization
└── Model Library with curated catalog
```

### Forward Kinematics Engine

The simulation computes link positions using **cumulative homogeneous transforms** through the kinematic tree:

$$ T_i^{world} = T_{parent}^{world} \cdot R(\hat{a}_i, q_i) \cdot T_{origin_i} $$

The rotation matrix \\( R(\hat{a}, q) \\) for axis-angle representation uses Rodrigues' formula:

$$ R(\hat{a}, q) = \cos(q) \cdot I + (1 - \cos q) \cdot \hat{a}\hat{a}^T + \sin(q) \cdot [\hat{a}]_\times $$

where \\( [\hat{a}]_\times \\) is the skew-symmetric matrix of the rotation axis. This propagates correctly through branching kinematic trees (essential for quadrupeds with 4 independent leg chains).

### Joint Control

Each actuated joint runs a PD controller:

$$ \tau = K_p (q_{target} - q_{current}) - K_d \dot{q}_{current} $$

with configurable gains and effort limits from the URDF specification.

### Drone Flight Dynamics

The quadcopter simulation models 6-DOF rigid body dynamics:

$$ \ddot{p} = \frac{1}{m}\left(R \cdot \begin{bmatrix}0\\0\\F_{total}\end{bmatrix} - F_{drag}\right) - g $$

$$ \dot{\omega} = I^{-1}(\tau_{rotors} - \omega \times I\omega) $$

with PID attitude stabilization and a rotor mixer for thrust allocation.

### Tech Stack

| Layer | Technology | Rationale |
|-------|-----------|-----------|
| Backend | Python 3.11 / FastAPI | Async WebSocket + fast prototyping |
| Physics | Custom engine (PyBullet-ready) | Zero-dependency, runs on free tier |
| Frontend | Three.js (vanilla) | No build step, instant load |
| Deployment | Render.com | Free, auto-deploy from GitHub |
| Testing | pytest (62 tests) | Enterprise-grade validation |
| Development | **Kiro by AWS** | AI-powered IDE for rapid iteration |

## What I Learned

1. **URDF is the universal interchange format** — Every robot manufacturer provides URDF files. Building on this standard gives instant access to thousands of models. USD (Omniverse's format) adds visual fidelity but lacks native kinematics support.

2. **Tree-structured robots are fundamentally harder** — My initial serial-chain assumption broke completely on the Unitree A1 quadruped. Implementing BFS traversal of kinematic trees taught me why robotics software distinguishes between serial manipulators and parallel/branching mechanisms.

3. **WebSocket streaming architecture** — Decoupling physics (240Hz) from rendering (60Hz) via a streaming buffer was essential for smooth visualization without blocking the simulation loop.

4. **Pure Python physics is surprisingly viable** — Without PyBullet's C++ compilation on Windows, I built a kinematic engine from scratch. For control algorithm validation (which is the primary use case), full contact dynamics aren't always necessary — accurate FK and PD control suffice.

5. **Kiro transforms development velocity** — Building an entire platform in 3 days with Clean Architecture, 62 tests, and deployment would have taken 2-3 weeks solo. Kiro handled scaffolding, debugging, and iteration while I focused on architecture and physics correctness.

## Challenges

### PyBullet Won't Compile on Windows

PyBullet requires Visual C++ build tools and doesn't ship prebuilt wheels for Python 3.11 on Windows. Rather than fight the toolchain, I implemented a **pure-Python kinematic engine** with proper rotation matrix propagation. This became a strength: it runs on any platform including Render's free tier (512MB RAM, no GPU).

### URDF Mesh Files Can't Load Cross-Origin

URDFs reference `.stl` and `.dae` mesh files for visual geometry, but browsers block cross-origin 3D asset loading. Solution: auto-generate geometric approximations from joint offsets (cylinders between joints, spheres at joint locations) with a graceful fallback. The physics remain accurate — only the visual is approximated.

### Quadrupeds Break Linear Assumptions

The Unitree A1's tree topology (trunk → 4 legs × 3 joints) caused the renderer to produce a nonsensical serial chain. Required rewriting the entire visualization pipeline as a proper **BFS tree traversal** that respects parent-child branching.

### Free-Tier Constraints

Render's 512MB RAM and sleep-after-inactivity policy forced me to keep the physics lightweight and stateless between requests. This architectural constraint actually improved the design — each simulation world is isolated and garbage-collectible.

## What's Next

- **AWS Bedrock** integration for AI robot programming ("move arm to grasp the red block")
- **ROS2 bridge** (scaffolded) for connecting real ROS nodes
- **Collision detection** with spatial hashing
- **Multi-user sessions** for collaborative robotics education
- **GPU physics** via AWS for production-grade contact dynamics

---

_Built by **Stafford Ho Sheng Xian** with [Kiro](https://kiro.dev) — the AI-powered development environment by AWS._

_Special thanks to the AWS team for providing credits through the Kiro BuildFest hackathon._
