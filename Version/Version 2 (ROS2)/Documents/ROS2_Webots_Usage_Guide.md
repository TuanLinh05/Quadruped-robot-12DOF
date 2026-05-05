# HƯỚNG DẪN KHỞI CHẠY HỆ THỐNG WEBOTS + ROS2 (UDP BRIDGE)

Tài liệu này tổng hợp toàn bộ quy trình từ A-Z để khởi động và điều khiển Robot chó 12-DOF bằng ROS2 thông qua WSL2 trên hệ điều hành Windows.

---

## BƯỚC 1: KHỞI ĐỘNG WEBOTS (TRÊN WINDOWS)

1. Mở phần mềm **Webots** trên Windows.
2. Mở file World mô phỏng của robot: `Webots_Simulation/worlds/quad_3dof_L1L2L3_4legs.wbt`.
3. Đảm bảo file điều khiển C `SMC_12DOF.c` đã được biên dịch thành công (Bấm vào biểu tượng **Bánh răng - Build** trên thanh công cụ của Webots).
4. Bấm nút **Play** (hoặc Run) để bắt đầu mô phỏng. 
   - *Lúc này, Robot sẽ đứng yên ở tư thế Stand (Mode 1).*
   - *Trên Console của Webots sẽ hiện dòng chữ: `--- HE THONG DIEU KHIEN SMC CHO ROBOT 12-DOF: ĐÃ KẾT NỐI UDP ROS2 ---`*

---

## BƯỚC 2: KHỞI CHẠY ROS2 BRIDGE (TRÊN WSL2 / UBUNTU)

Mở **Terminal số 1** của Ubuntu (WSL2) và thực hiện các lệnh sau:

```bash
# 1. Kích hoạt môi trường ROS2 (Sửa 'humble' thành phiên bản bạn đang dùng nếu cần)
source /opt/ros/humble/setup.bash

# 2. Bắt buộc: Ép ROS2 dùng mạng cục bộ để sửa lỗi Multicast của WSL2
export ROS_LOCALHOST_ONLY=1

# 3. Di chuyển đến thư mục chứa code Bridge (thông qua đường dẫn /mnt/c của Windows)
cd "/mnt/c/Users/ADMIN/Downloads/Final Dog/ROS2_Bridge"

# 4. Chạy file Python Node
python3 ros2_udp_bridge.py
```
*Ghi chú: Khi chạy thành công, Terminal sẽ báo đã tìm thấy IP của Windows Host và đang lắng nghe topic `/cmd_vel`.*

---

## BƯỚC 3: ĐIỀU KHIỂN ROBOT BẰNG ROS2

Mở **Terminal số 2** của Ubuntu (WSL2), giữ nguyên Terminal 1 đang chạy Bridge.

```bash
# 1. Kích hoạt môi trường và sửa lỗi mạng
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
```

**Các câu lệnh điều khiển (Copy/Paste vào Terminal 2):**

*👉 **Mode 4 (Trot - Đi nước kiệu tới trước):***
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
```

*👉 **Mode 8 (Crab Walk - Đi bò ngang như cua):***
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {y: 0.5}}"
```

*👉 **Mode 7 (Roll Sway - Lắc lư nghiêng người sang hai bên):***
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"
```

*👉 **Dừng lại (Chuyển về Stand):***
Bấm `Ctrl + C` ở Terminal 2 để ngắt lệnh gửi. Hệ thống sẽ tự động bắt diện vận tốc bằng 0 và trả robot về tư thế đứng yên.

---

## GIẢI THÍCH KIẾN TRÚC & KHẮC PHỤC SỰ CỐ
- **Lỗi mạng WSL2 UDP:** Nếu Node ROS2 không truyền được tín hiệu sang Webots, lý do thường là WSL2 chặn các gói tin UDP bắn vào `127.0.0.1`. File `ros2_udp_bridge.py` đã tự động khắc phục điều này bằng cách dùng lệnh `ip route` tìm địa chỉ IP Card mạng vEthernet (Gateway) của Windows và gửi gói tin thẳng vào đó.
- **Lỗi Waiting for matching subscription:** Do WSL2 không hỗ trợ Multicast `lo`, các Node ROS2 không thể tự tìm thấy nhau. Lệnh `export ROS_LOCALHOST_ONLY=1` là "liều thuốc tiên" để ép chúng liên kết trong mạng Loopback nội bộ.
