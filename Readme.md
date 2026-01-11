
# PX4 ROS 2 EKF + PID Offboard Control (SITL)

## 1. Project Overview

This project implements **a complete perception–control pipeline** for a PX4 multicopter using **ROS 2 (Humble)** and **PX4 SITL**.

The pipeline consists of:

1. **Custom EKF (Extended Kalman Filter)** written from scratch
2. **ROS 2 node** subscribing to PX4 IMU + position topics
3. **State estimation publishing** (`/ekf_data`)
4. **PID-based offboard controller**
5. **PX4 Offboard control interface**
6. **Full ROS 2 ↔ PX4 communication via Micro XRCE-DDS Agent**

The objective is **learning and validation**, not replacing PX4’s EKF.  
All design decisions are explicit and educational.

---

## 2. System Architecture

```
PX4 SITL
 ├─ IMU (sensor_combined)
 ├─ Local Position (vehicle_local_position_v1)
 │
 ▼
ROS 2 EKF Node
 ├─ State prediction
 ├─ Covariance propagation
 ├─ Measurement updates
 └─ Publishes /ekf_data
 │
 ▼
ROS 2 PID Offboard Node
 ├─ Roll / Pitch PID
 ├─ Cascaded Altitude PID
 └─ PX4 Offboard Commands
 │
 ▼
PX4 Commander + Attitude Controller
```

---

## 3. Coordinate Frames (CRITICAL)

PX4 uses **NED / FRD conventions**:

| Quantity | Frame | Sign |
|--------|------|------|
| Z (altitude) | Down | +ve down |
| Thrust | Body FRD | Negative Z |
| Roll / Pitch | Body | Right-hand |

**Important consequence:**  
Hover thrust must be **negative in Z body axis**.

---

## 4. Software Requirements

### OS
- Ubuntu 22.04

### ROS 2
- ROS 2 Humble

Install:
```bash
sudo apt install ros-humble-desktop
```

### PX4 SITL
```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl gz_x500
```

---

## 5. Micro XRCE-DDS Agent 

PX4 communicates with ROS 2 via **Micro XRCE-DDS**.
To setup this refer to px4 documentation 
### Run Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

⚠️ PX4 will **NOT accept offboard commands** without this agent.

---

## 6. ROS 2 Workspace Setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# place your package here
cd ..
colcon build --packages-select review
source install/setup.bash
```

Always source:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 7. EKF Node Details

### Subscriptions

| Topic | Purpose |
|-----|--------|
| `/fmu/out/sensor_combined` | Raw IMU |
| `/fmu/out/vehicle_local_position_v1` | Z, Vz (altimeter proxy) |

### Publications

| Topic | Content |
|------|--------|
| `/ekf_data` | `[roll, pitch, yaw_rate, z, vz]` |

---

## 8. EKF Design Summary

### State Vector
```
x = [φ, θ, r, z, v_z]
```

### Why EKF?
- Sensors noisy
- Physics model imperfect
- Need uncertainty-aware fusion

### Key Learnings
- EKF **does not need to measure every state**
- Covariance couples states
- Unmeasured states improve via correlation

---

## 9. PID Offboard Controller

### Control Structure

```
Position (z)
 ↓
Velocity (vz)
 ↓
Thrust
```

Roll & pitch directly stabilized via PID.

### Why Cascaded?
- Stable
- Industry standard
- Prevents thrust oscillation

---

## 10. PX4 Offboard Mental Model (VERY IMPORTANT)

Think of it as **three layers**:

1. `VehicleCommand`
   → *"I want control"*

2. `OffboardControlMode`
   → *"I am alive, this is how I control"*

3. `VehicleAttitudeSetpoint`
   → *"Here are actual commands"*

⚠️ All **three are mandatory**

---

## 11. Common Failure Modes

| Symptom | Cause |
|------|------|
| Motors not spinning | Offboard heartbeat missing |
| Disarms instantly | No setpoints before mode switch |
| No response | XRCE Agent not running |
| Thrust ignored | Wrong sign (FRD frame) |

---

## 12. Verification Checklist

- [ ] MicroXRCEAgent running
- [ ] EKF publishing `/ekf_data`
- [ ] OffboardControlMode published >2Hz
- [ ] Vehicle armed
- [ ] Mode switched to OFFBOARD
- [ ] Thrust negative Z

---

## 13. Conclusions & Learnings

### EKF
- Covariance is as important as state
- Measurement noise tuning dominates performance
- Vertical estimation sensitive to accelerometer bias

### Control
- Hover thrust must be calibrated
- Cascaded control improves stability
- Offboard safety requires continuous heartbeat

### PX4
- Offboard is **fail-safe by design**
- Strict separation of authority & control
- ROS integration enforces discipline

---

## 14. Next Steps

- Replace PID with MPC
- Use full rotation matrix in EKF
- Bias estimation
- Hardware deployment

---

## 15. Final Note

This project demonstrates **true understanding of estimation + control**, not copy-paste usage.

If you understand why **nothing moves when one message is missing**, you understand PX4.
