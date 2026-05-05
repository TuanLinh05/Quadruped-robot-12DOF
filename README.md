# 🐕 12-DOF Quadruped Robot: Sliding Mode Control (SMC) with ROS2 & Webots

![Webots](https://img.shields.io/badge/Webots-R2025a-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)
![MATLAB](https://img.shields.io/badge/MATLAB-R2023-orange)
![C](https://img.shields.io/badge/Language-C/Python-yellow)

A highly optimized, 12 Degrees of Freedom (DOF) quadruped robot simulation developed in Webots. This project implements a custom non-linear **Sliding Mode Controller (SMC)** based on Euler-Lagrange dynamics, a **6th-order Bézier curve** gait planner, and features seamless **ROS2 integration** via a custom UDP bridge to overcome WSL2 networking limitations.

---

## 🚀 Key Features

*   **Non-linear Dynamics & Control:** Complete derivation of Euler-Lagrange equations (Mass, Coriolis, Gravity matrices) mapped into a robust Sliding Mode Control (SMC) law with chattering reduction (`sat()` function).
*   **Hybrid Control Architecture:** Utilizes highly stiff Position Control for Yaw joints (to resist lateral friction) and flexible Torque Control (SMC) for Pitch/Knee joints.
*   **Advanced Gait Planning:** Implements a 6th-order Bézier curve trajectory generator for smooth foot placement, eliminating ground-impact spikes.
*   **Multi-Mode Execution:** Supports 8 different locomotion modes including *Stand, Squat, Belly Dance, Trot, Pace, Gallop, Roll Sway,* and *Crab Walk*.
*   **ROS2 Teleoperation Bridge:** Real-time mode switching via ROS2 `/cmd_vel` topics. Features a custom Python UDP bridge designed to bypass WSL2 multicast and loopback isolation issues.
*   **Real-time MATLAB Telemetry:** A 50Hz UDP stream from the C-Controller to MATLAB Simulink/Scripts for live tracking error and trajectory plotting.

---

## 🏗️ System Architecture

The project employs an **Edge-Computing / Hardware-in-the-Loop** architecture. To maintain the critical 1000Hz frequency required for non-linear SMC stability, the core controller is embedded entirely in C. ROS2 acts as the High-Level Navigation brain.

```text
[ ROS2 Node (Ubuntu/WSL2) ] --(Twist /cmd_vel)--> [ Python UDP Bridge ]
                                                          |
                                                    (UDP Port 5556)
                                                          |
[ MATLAB Dashboard ] <--(UDP Port 5555)-- [ Webots C-Controller (1000Hz) ]
```

---

## 📂 Project Structure

```text
📦 Final Dog
 ┣ 📂 Backup/                 # Old codebase iterations
 ┣ 📂 Documents/              # Technical reports and math derivations
 ┃ ┣ 📜 Algorithm_Math.md
 ┃ ┣ 📜 Euler_Lagrange_Derivation.md
 ┃ ┗ 📜 ROS2_Webots_Usage_Guide.md
 ┣ 📂 MATLAB_Scripts/         # Telemetry plotting scripts
 ┣ 📂 ROS2_Bridge/            # Python bridge for WSL2-Windows communication
 ┃ ┗ 📜 ros2_udp_bridge.py
 ┣ 📂 Webots_Simulation/      # The core simulation environment
 ┃ ┣ 📂 controllers/
 ┃ ┃ ┗ 📂 SMC_12DOF/          # C-Core Controller (SMC, IK, Gait Planner)
 ┃ ┃   ┣ 📜 SMC_12DOF.c
 ┃ ┃   ┗ 📜 math_utils.c / .h
 ┃ ┗ 📂 worlds/               # Webots world files
 ┗ 📜 README.md               # You are here
```

---

## ⚙️ Prerequisites

1.  **Windows Host:** Webots R2023+ installed.
2.  **WSL2 (Ubuntu 22.04+):** ROS2 Humble installed.
3.  **MATLAB (Optional):** For real-time telemetry plotting.

---

## 🎮 Quick Start Guide

### Step 1: Start the Webots Simulation (Windows)
1. Open `Webots_Simulation/worlds/quad_3dof_L1L2L3_4legs.wbt` in Webots.
2. Click the **Build (Gear icon)** to compile the `SMC_12DOF` controller.
3. Click **Play**. The robot will initialize in *Stand* mode.

### Step 2: Launch the ROS2 Bridge (WSL2 Terminal 1)
Since WSL2 blocks UDP loopback to Windows, you must run the bridge script which automatically finds the Windows vEthernet Gateway IP.

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Fix WSL2 Multicast bug
export ROS_LOCALHOST_ONLY=1

# Run the Bridge
cd "/mnt/c/Users/ADMIN/Downloads/Final Dog/ROS2_Bridge"
python3 ros2_udp_bridge.py
```

### Step 3: Teleoperate the Robot (WSL2 Terminal 2)
Open a new WSL2 terminal to publish velocity commands:

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1

# 1. Trot Forward (Mode 4)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

# 2. Crab Walk / Walk Sideways (Mode 8)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {y: 0.5}}"

# 3. Roll Sway / Belly Dance (Mode 7)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"

# 4. Stop (Press Ctrl+C to stop publishing. The bridge will return to Stand Mode).
```

---

## 📚 Documentation
For a deep dive into the mathematics and code design, please refer to the files in the `Documents/` directory:
- [Euler-Lagrange Derivation](Documents/Euler_Lagrange_Derivation.md): Step-by-step proof of the $M, C, G$ matrices.
- [Algorithm Math](Documents/Algorithm_Math.md): Detailed explanation of the 6th-order Bézier trajectory and IK.

---
*Developed as a Robotics Control Systems Project.*
