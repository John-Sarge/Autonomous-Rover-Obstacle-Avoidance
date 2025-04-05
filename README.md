# 🤖 Autonomous Rover Obstacle Avoidance System

This project implements an autonomous obstacle avoidance system for a differential drive (skid-steer) rover using an **OAK-D depth camera** and **MAVLink** communication via **mavlink-router**. It uses depth data to detect obstacles in real-time and issues RC override commands to a Pixhawk (or similar) running ArduPilot.

---

## 📸 System Overview

- **Depth Sensing:** Powered by the Luxonis OAK-D stereo camera using DepthAI SDK.
- **Obstacle Detection:** Custom-defined Regions of Interest (ROIs) in the camera view allow fine-tuned detection zones.
- **Reactive Movement:** Based on proximity and position of detected obstacles, the rover can stop, turn, or reverse.
- **Control Interface:** Sends RC override commands to the rover through MAVLink protocol via `pymavlink`.

---

## 🚀 Features

- Real-time obstacle detection using spatial depth data.
- Tunable obstacle response logic (`STOP`, `TURN_LEFT`, `TURN_RIGHT`, `REVERSE`).
- Safety-aware geometric check to prevent unsafe turns near obstacles.
- Configurable ROIs to match your rover’s field of view and size.
- Graceful exit on SIGINT/SIGTERM (Ctrl+C or shutdown).

---

## 🧰 Requirements

### Hardware

- Pixhawk-compatible autopilot with ArduPilot Rover firmware
- OAK-D or OAK-D Lite stereo camera
- Companion computer (e.g., Raspberry Pi, Jetson Nano)

### Software

- Python 3.7+
- `depthai`
- `pymavlink`
- `mavlink-router`

Install dependencies:

```bash
pip install depthai pymavlink
```

---

## ⚙️ Configuration

Before running the script, tune the following parameters as needed:

### MAVLink Settings

| Parameter | Description |
|----------|-------------|
| `MAVLINK_ROUTER_HOST` | IP address where mavlink-router is listening (usually `127.0.0.1`) |
| `MAVLINK_ROUTER_PORT` | UDP port configured in `mavlink-router` (e.g., `14551`) |
| `MAVLINK_SYSTEM_ID` / `COMPONENT_ID` | Target system/component IDs for your Pixhawk |
| `GCS_SOURCE_SYSTEM` | Source ID for this script (typically `255`) |

### DepthAI Settings

| Parameter | Description |
|----------|-------------|
| `DEPTH_RESOLUTION` | Mono camera resolution (`THE_400_P`, `THE_720_P`, etc.) |
| `DEPTH_RATE_HZ` | Frame rate for depth processing |
| `CONFIDENCE_THRESHOLD` | Filter for noisy pixels (suggested: `220`) |

### Obstacle Detection

| Parameter | Description |
|----------|-------------|
| `OBSTACLE_THRESHOLD_M` | Trigger distance for obstacle detection (e.g., `0.5m`) |
| `OBSTACLE_ACTION` | Action taken on center obstacle (`STOP`, `TURN_LEFT`, etc.) |

### Motor Control

| Parameter | Description |
|----------|-------------|
| `DRIVE_NEUTRAL_PWM` | PWM value for motor stop (usually `1500`) |
| `DRIVE_FORWARD_PWM` | PWM value for forward motion |
| `DRIVE_REVERSE_PWM` | PWM value for reverse motion |
| `RC_CHAN_LEFT_THROTTLE_IDX` / `RIGHT_THROTTLE_IDX` | Channel index for left/right motors |

### Rover Geometry

| Parameter | Description |
|----------|-------------|
| `ROVER_WIDTH_M` | Full width of rover in meters |
| `SIDE_CLEARANCE_M` | Buffer distance to prevent unsafe turns |

---

## 🧠 How It Works

1. **DepthAI pipeline** is initialized with stereo cameras and spatial ROI calculations.
2. **ROIs** are configured to detect obstacles in the center, left, and right regions.
3. **RC override commands** are sent based on obstacle distance and location.
4. **Geometric checks** ensure turns won’t cause collisions with nearby obstacles.
5. The rover reacts accordingly by stopping, turning, or reversing.

---

## 🛠️ Usage

1. Make sure `mavlink-router` is running and properly configured.
2. Connect the OAK-D camera to your companion computer.
3. Arm your rover and switch to **Manual** or **Acro** mode.
4. Run the script:

```bash
python3 obstacle_avoidance.py
```

Press `Ctrl+C` to stop safely at any time.

---

## 🧪 Testing & Tuning

- Use **visual tools** to debug ROIs and adjust their size and location.
- Start with **low forward PWM** and **small ROIs**.
- Increase `SIDE_CLEARANCE_M` if the rover gets too close to obstacles on turns.
- Test in simulation with SITL + MAVProxy if hardware is unavailable.

---

## 🧾 License

This project is released under the MIT License.

---

## 🙌 Credits

Built using:
- [DepthAI](https://docs.luxonis.com/)
- [ArduPilot](https://ardupilot.org/)
- [pymavlink](https://www.ardusub.com/developers/pymavlink.html)
- [mavlink-router](https://github.com/mavlink-router/mavlink-router)


