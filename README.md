# 🐕 12-DOF Quadruped Robot – Sliding Mode Control in Webots & ROS 2

![Webots](https://img.shields.io/badge/Webots-R2023+-C62828)
![ROS2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![MATLAB](https://img.shields.io/badge/MATLAB-Dashboard-E16737)
![Language](https://img.shields.io/badge/Language-C%20%7C%20Python%20%7C%20MATLAB-555555)
![Control](https://img.shields.io/badge/Control-Sliding%20Mode-6A1B9A)

<a id="english"></a>**🇬🇧 English** · [🇻🇳 Tiếng Việt](#tieng-viet)

Simulation and control of a **12-DOF quadruped robot** in **Webots**. A torque-based **Sliding Mode Controller (SMC)**, derived from the Euler–Lagrange dynamics, runs at **1 kHz** in C. Gaits are generated with **6th-order Bézier curves**, the robot is driven through **ROS 2 `/cmd_vel`**, and foot trajectories are streamed live to a **MATLAB dashboard**.

---

## ✨ Highlights

- **Non-linear control:** torque-based SMC with a boundary layer to reduce chattering, based on Euler–Lagrange dynamics and inverse kinematics.
- **Smooth gaits:** 6th-order Bézier foot trajectories with zero-jerk touchdown.
- **Hybrid control:** position control locks the hip yaw joints for traction, and SMC torque control drives the hip pitch and knee joints.
- **8 motion modes:** Stand, Squat, Belly Dance, Trot, Pace, Gallop, Roll Sway, Crab Walk.
- **ROS 2 teleoperation:** `/cmd_vel` → Python UDP bridge → Webots C controller.
- **Real-time telemetry:** foot positions of all 4 legs sent over UDP at 50 Hz to MATLAB.

## 🏗️ Architecture

```text
[ ROS 2 node ] --(Twist /cmd_vel)--> [ Python UDP bridge ] --(UDP 5556)--> [ Webots C controller · SMC 1 kHz ]
                                                                                        |
                                              [ MATLAB dashboard ] <------(UDP 5555)----+
```

## 📂 Repository structure

```text
Quadruped-robot-12DOF/
├── quadruped_ros2/              # Main project (Webots + C controller + bridge + MATLAB)
│   ├── Documents/               # Math derivations: Euler–Lagrange, algorithms, usage guide
│   ├── MATLAB_Scripts/          # Real-time dashboards and plotting
│   ├── ROS2_Bridge/             # ros2_udp_bridge.py
│   └── Webots_Simulation/       # World file + SMC_12DOF C controller
└── ros2_ws/src/quadruped_ros2/  # ROS 2 package (Python): gait planner, kinematics, SMC math, launch files
```

## 🚀 Quick start

> Requirements: Ubuntu 22.04, Webots R2023+, ROS 2 Humble, MATLAB (optional).

1. Open `quadruped_ros2/Webots_Simulation/worlds/quad_3dof_L1L2L3_4legs.wbt` in Webots, build the `SMC_12DOF` controller and press **Play**.
2. In a new terminal, start the bridge:
   ```bash
   source /opt/ros/humble/setup.bash
   cd quadruped_ros2/ROS2_Bridge
   python3 ros2_udp_bridge.py
   ```
3. Drive the robot from another terminal:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"   # trot forward
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {y: 0.5}}"   # crab walk
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"  # roll sway
   ```
   Press `Ctrl+C` to stop: the bridge puts the robot back in **Stand** mode.
4. *(Optional)* Run `MATLAB_Scripts/smc_dashboard.m` to watch the foot trajectories live.

**ROS 2 package version** (`ros2_ws`):
```bash
cd ros2_ws
colcon build --symlink-install --packages-select quadruped_ros2
source install/setup.bash
ros2 launch quadruped_ros2 quadruped_webots.launch.py
```

📖 The full guide, the math and the mode list are in [`quadruped_ros2/README.md`](quadruped_ros2/README.md) and [`quadruped_ros2/Documents`](quadruped_ros2/Documents).

## 🔗 Related

- [ROS2_Webots_DOG](https://github.com/TuanLinh05/ROS2_Webots_DOG) – SMC gait control for the Unitree Go2 model in ROS 2 + Webots.

---

<a id="tieng-viet"></a>

## 🇻🇳 Tiếng Việt

[🇬🇧 English](#english) · **🇻🇳 Tiếng Việt**

Mô phỏng và điều khiển **robot bốn chân 12 bậc tự do** trong **Webots**. Bộ điều khiển **trượt (Sliding Mode Control – SMC)** theo mô-men, xây dựng từ phương trình động lực học Euler–Lagrange, chạy ở **1 kHz** bằng C. Dáng đi được tạo bằng **đường cong Bézier bậc 6**. Robot được điều khiển qua **ROS 2 `/cmd_vel`**, còn quỹ đạo bàn chân được truyền trực tiếp lên **dashboard MATLAB**.

### ✨ Điểm nổi bật

- **Điều khiển phi tuyến:** SMC theo mô-men, có lớp biên (boundary layer) để giảm chattering, dựa trên động lực học Euler–Lagrange và động học ngược.
- **Dáng đi mượt:** quỹ đạo bàn chân Bézier bậc 6, không giật (zero jerk) khi chạm đất.
- **Điều khiển lai:** điều khiển vị trí khóa cứng khớp yaw ở hông để tăng độ bám. SMC điều khiển mô-men cho khớp pitch ở hông và khớp gối.
- **8 chế độ chuyển động:** Stand, Squat, Belly Dance, Trot, Pace, Gallop, Roll Sway, Crab Walk.
- **Điều khiển qua ROS 2:** `/cmd_vel` → cầu nối UDP bằng Python → bộ điều khiển C trong Webots.
- **Telemetry thời gian thực:** tọa độ 4 bàn chân gửi qua UDP 50 Hz lên MATLAB.

Sơ đồ kiến trúc và cấu trúc thư mục: xem phần tiếng Anh ở trên.

### 🚀 Khởi chạy nhanh

> Yêu cầu: Ubuntu 22.04, Webots R2023 trở lên, ROS 2 Humble, MATLAB (không bắt buộc).

1. Mở `quadruped_ros2/Webots_Simulation/worlds/quad_3dof_L1L2L3_4legs.wbt` trong Webots, build controller `SMC_12DOF` rồi bấm **Play**.
2. Mở terminal mới và chạy cầu nối: `cd quadruped_ros2/ROS2_Bridge && python3 ros2_udp_bridge.py` (nhớ `source /opt/ros/humble/setup.bash` trước).
3. Ở terminal khác, gửi lệnh `/cmd_vel` như ví dụ ở phần tiếng Anh. Bấm `Ctrl+C` để dừng: robot sẽ về chế độ **Stand**.
4. *(Tùy chọn)* Chạy `MATLAB_Scripts/smc_dashboard.m` để xem quỹ đạo bàn chân trực tiếp.

📖 Hướng dẫn đầy đủ, phần toán và danh sách chế độ nằm trong [`quadruped_ros2/README.md`](quadruped_ros2/README.md) và thư mục [`quadruped_ros2/Documents`](quadruped_ros2/Documents).

---

<p align="center">Made by <a href="https://github.com/TuanLinh05">Vu Tuan Linh</a> · HCMUT</p>
