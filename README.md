# 🏎️ RoboRacer Workspace

> **A high-adrenaline "battle of algorithms"** — 1:10 scale autonomous vehicles racing head-to-head.

This workspace contains the full simulation and control stack for the **RoboRacer** competition, built on **ROS 2 Jazzy** and **Gazebo Sim (Harmonic)**.

---

## 📁 Project Structure

```
RoboRacer_WS/
├── src/
│   ├── roboracer/                    # Core racing packages
│   │   ├── roboracer_description/    # URDF, meshes, and sensors
│   │   ├── roboracer_gazebo/         # Gazebo world, controllers, bridge
│   │   └── roboracer_interfaces/     # Custom messages and services
│   ├── racing_logic/                 # YOUR algorithms go here
│   │   └── (Pure Pursuit, MPC, Obstacle Avoidance)
│   └── support/                      # Reference examples from class
├── docs/                             # Documentation and contest rules
├── scripts/                          # Helper scripts (build, launch)
└── README.md
```

---

## 🚀 Quick Start

### 1. Install Dependencies
```bash
sudo apt update && \
sudo apt install -y ros-jazzy-ros2-control \
                    ros-jazzy-ros2-controllers \
                    ros-jazzy-gz-ros2-control \
                    ros-jazzy-ros-gz \
                    ros-jazzy-ros-gz-bridge \
                    ros-jazzy-joint-state-publisher \
                    ros-jazzy-robot-state-publisher \
                    ros-jazzy-xacro \
                    ros-jazzy-joy
```

### 2. Build
```bash
./scripts/build.sh
```

### 3. Launch Simulation
```bash
./scripts/launch_sim.sh
```

---

## 🎯 Competition Tasks

| Task | Description | Status |
| :--- | :--- | :---: |
| **Time Trial** | Solo lap — shortest time wins | ⬜ |
| **Head-to-Head** | Race with collision avoidance | ⬜ |
| **Dynamic Obstacles** | Overtake without crashing | ⬜ |

---

## 🕹️ Control Interface

The vehicle accepts commands on `/qcar_sim/user_command` (`geometry_msgs/Vector3Stamped`):
- `vector.x` → Linear velocity (m/s)
- `vector.y` → Steering angle (rad)

---

## 👥 Team

- **Tecnológico de Monterrey**
- Advisor: Dr. Daniel Sosa-Ceron / Dr. Jorge A. Reyes-Avendaño
