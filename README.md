```markdown
# OAK-D MAVLink Rover Obstacle Avoidance

This Python script enables autonomous obstacle avoidance for an ArduPilot Rover using depth data from an OAK-D camera. It processes spatial information from the camera, identifies obstacles within defined Regions of Interest (ROIs), and sends `RC_CHANNELS_OVERRIDE` MAVLink commands via `mavlink-router` to control a differential drive (skid steer) rover, enabling reactive maneuvers.

A key feature is a geometric safety check that prevents turns into obstacles that are detected too close to the rover's side, opting for a reverse maneuver instead.

**Author:** John Seargeant

---

## Table of Contents

1.  [Overview](#overview)
2.  [Features](#features)
3.  [Hardware Requirements](#hardware-requirements)
4.  [Software Prerequisites](#software-prerequisites)
5.  [Installation](#installation)
6.  [ArduPilot Configuration](#ardupilot-configuration)
7.  [`mavlink-router` Configuration](#mavlink-router-configuration)
8.  [Script Configuration](#script-configuration)
    * [MAVLink Settings](#mavlink-settings)
    * [OAK-D / DepthAI Settings](#oak-d--depthai-settings)
    * [Regions of Interest (ROIs)](#regions-of-interest-rois)
    * [Obstacle Avoidance Logic](#obstacle-avoidance-logic)
    * [Differential Drive PWM Tuning](#differential-drive-pwm-tuning)
    * [RC Channel Override Assignment](#rc-channel-override-assignment)
    * [Rover Geometry](#rover-geometry)
9.  [Usage](#usage)
10. [How it Works](#how-it-works)
    * [DepthAI Pipeline](#depthai-pipeline)
    * [MAVLink Communication](#mavlink-communication)
    * [Obstacle Detection & Logic](#obstacle-detection--logic)
    * [Geometric Turn Safety Check](#geometric-turn-safety-check)
11. [Troubleshooting / Notes](#troubleshooting--notes)
12. [License](#license)

---

## Overview

This script acts as a companion computer process that interfaces an OAK-D stereo camera with an ArduPilot flight controller (like a Pixhawk) running Rover firmware. It aims to provide basic reactive obstacle avoidance capabilities suitable for indoor or relatively slow-moving outdoor environments.

* **Input:** Depth data from OAK-D.
* **Processing:** Calculates distances to obstacles in predefined zones (left, center, right).
* **Output:** MAVLink `RC_CHANNELS_OVERRIDE` messages to directly control the left and right motors of a differential drive rover.
* **Communication:** Relies on `mavlink-router` to bridge communication between this script and the flight controller.

## Features

* Utilizes OAK-D stereo depth perception for obstacle detection.
* Configurable Regions of Interest (ROIs) for targeted obstacle checking.
* Sends MAVLink `RC_CHANNELS_OVERRIDE` for direct motor control of differential drive rovers.
* Reactive avoidance logic:
    * Move forward when clear.
    * Perform configurable action (Stop, Turn Left/Right, Reverse) for center obstacles.
    * Turn away (counter-spin) from side obstacles.
* **Geometric Turn Safety Check:** Prevents initiating a turn if the detected side obstacle is already within the rover's physical turning path, triggering a reverse action instead.
* Configurable thresholds, PWM values, and behavior via constants in the script.
* Graceful shutdown using signal handling (Ctrl+C).

## Hardware Requirements

1.  **OAK-D Camera:** (OAK-D, OAK-D Lite, OAK-D Pro, etc.) Connected to the companion computer.
2.  **ArduPilot Flight Controller:** Pixhawk or compatible board running ArduPilot Rover firmware.
3.  **Rover Platform:** Configured for differential drive / skid steering (two motors controlling left/right sides independently).
4.  **Companion Computer:** A device capable of running Python 3, the DepthAI library, and `pymavlink` (e.g., Raspberry Pi 4/5, Jetson Nano, NUC).
5.  **Telemetry Radio / Wired Link:** Connection between the flight controller and the companion computer for MAVLink communication (e.g., USB, Serial, Wi-Fi).

## Software Prerequisites

1.  **Python 3:** Installed on the companion computer.
2.  **DepthAI Python Library:** `pip install depthai numpy`
3.  **Pymavlink Python Library:** `pip install pymavlink`
4.  **`mavlink-router`:** Installed and running on the companion computer. ([mavlink-router GitHub](https://github.com/mavlink-router/mavlink-router))
5.  **ArduPilot Rover Firmware:** Flashed onto the flight controller and configured (see below).
6.  **(Linux)** OAK device udev rules might need to be set up for non-root access. Follow [Luxonis documentation](https://docs.luxonis.com/en/latest/pages/troubleshooting/#udev-rules-fix).

## Installation

1.  **Clone the repository (or download the script):**
    ```bash
    git clone <your-repo-url>
    cd <your-repo-directory>
    ```
2.  **Install Python dependencies:**
    ```bash
    pip install -r requirements.txt
    # Or manually:
    # pip install depthai numpy pymavlink
    ```
3.  **Install and configure `mavlink-router`** (See section below).
4.  **Configure ArduPilot parameters** (See section below).
5.  **Adjust script configuration constants** (See section below).

## ArduPilot Configuration

Ensure your ArduPilot Rover firmware is configured correctly using a Ground Control Station (e.g., Mission Planner, QGroundControl):

1.  **Firmware:** Use the latest stable ArduPilot **Rover** firmware.
2.  **Frame Type:** `FRAME_CLASS = 2` (Skid Steer).
3.  **Steering Type:** `PILOT_STEER_TYPE = 1` (Two Paddles Input for skid steer, maps Throttle/Steering to Left/Right motors). Alternatively, if direct RC Override mapping is preferred, ensure `SERVOx_FUNCTION` matches expectations (see below).
4.  **Motor Outputs:** Configure `SERVO1_FUNCTION` and `SERVO3_FUNCTION` (or whichever outputs your motors are connected to) for motor control. **Crucially**, for this script's default RC Override mapping (`RC_CHAN_LEFT_THROTTLE_IDX = 1`, `RC_CHAN_RIGHT_THROTTLE_IDX = 0`), you likely need:
    * `SERVO1_FUNCTION = 73` (Throttle Left)
    * `SERVO3_FUNCTION = 74` (Throttle Right)
    * **Verify this mapping!** Use the RC calibration or Servo Output screen in your GCS to confirm that MAVLink RC Override Channel 1 controls the Right motor and Channel 2 controls the Left motor. Adjust `RC_CHAN_LEFT_THROTTLE_IDX` and `RC_CHAN_RIGHT_THROTTLE_IDX` in the script if your setup differs.
5.  **RC Failsafe:** Configure appropriately. RC Override requires the flight controller to be receiving valid RC signals (or for RC Failsafe to be disabled for testing, which is **NOT recommended** for general use).
6.  **Mode:** The script relies on `RC_CHANNELS_OVERRIDE`. This typically works best in modes like **Manual** or **Acro** where the pilot has direct control (which the script is overriding). It may not function as expected in autonomous modes like Auto, Guided, or RTL without further ArduPilot configuration (e.g., using `SET_ATTITUDE_TARGET` or specific companion computer integrations).

## `mavlink-router` Configuration

`mavlink-router` acts as a message broker. It needs to listen for MAVLink messages from the flight controller and route them, while also listening for commands *from this script* and forwarding them *to* the flight controller.

A typical `mavlink-router` configuration file (`main.conf` or similar) might include:

```ini
[General]
TcpServerPort = 5760 # Example: Allow GCS connection over TCP
ReportStats = false

# Endpoint for the Flight Controller (adjust /dev/tty* or IP/Port as needed)
[UartEndpoint FC]
Device = /dev/ttyACM0 # Or your serial port
Baud = 115200      # Or your telemetry baud rate

# Endpoint FOR THIS SCRIPT TO SEND COMMANDS TO
# This script sends UDP packets TO mavlink-router on this port.
# Mode=Server means mavlink-router LISTENS on this port.
[UdpEndpoint CompanionScript]
Mode = Server
Address = 127.0.0.1 # Listen only on localhost
Port = 14551        # MUST MATCH MAVLINK_ROUTER_PORT in the Python script

# (Optional) Endpoint for a Ground Control Station
# [UdpEndpoint GCS]
# Mode = Normal
# Address = 192.168.1.100 # IP of your GCS machine
# Port = 14550
```

**Ensure `mavlink-router` is running before starting the Python script.**

```bash
mavlink-routerd -c /path/to/your/main.conf
```

## Script Configuration

Key parameters are defined as constants near the top of the Python script. **Adjust these carefully based on your specific hardware and environment.**

### MAVLink Settings

* `MAVLINK_ROUTER_HOST = '127.0.0.1'`: IP where `mavlink-router` is listening. Usually localhost if running on the same companion computer.
* `MAVLINK_ROUTER_PORT = 14551`: UDP port `mavlink-router` is listening on for commands *from this script*. Must match the `Port` in the `UdpEndpoint` section of `mavlink-router.conf` that has `Mode = Server`.
* `MAVLINK_SYSTEM_ID = 1`: Target System ID of your flight controller (usually 1).
* `MAVLINK_COMPONENT_ID = 1`: Target Component ID of your flight controller (usually 1 for Autopilot).
* `GCS_SOURCE_SYSTEM = 255`: System ID this script uses when sending MAVLink messages.

### OAK-D / DepthAI Settings

* `DEPTH_RESOLUTION = dai.MonoCameraProperties.SensorResolution.THE_400_P`: Resolution for stereo cameras (`THE_400_P`, `THE_720_P`, `THE_800_P`). Lower resolutions are faster but less detailed. 400p is often a good balance.
* `DEPTH_RATE_HZ = 15`: Target frames per second for depth processing. Higher demands more resources.
* `CONFIDENCE_THRESHOLD = 220`: (0-255) Minimum confidence required for a depth pixel to be considered valid. Higher values filter more noise but may discard valid points in challenging lighting/texture conditions. **Tunable.**
* `SPATIAL_CALC_METHOD = dai.SpatialLocationCalculatorAlgorithm.MIN`: Method to determine the representative distance within an ROI (`MIN`, `AVERAGE`, `MEDIAN`, `MODE`). `MIN` is sensitive to the closest point (good for obstacle detection, but susceptible to noise).

### Regions of Interest (ROIs)

* `ROIS = [...]`: Defines named rectangular areas in the normalized camera view (0.0-1.0 for X and Y) where the script actively looks for obstacles.
    * `(0,0)` is top-left, `(1,1)` is bottom-right.
    * Adjust the `Xmin, Ymin, Xmax, Ymax` coordinates based on your camera's mounting angle, field of view, and where you need detection. Visualizing the ROIs overlaid on the camera feed during testing is highly recommended.
    * The default setup defines `center`, `left`, and `right` zones in the lower-middle part of the view.

### Obstacle Avoidance Logic

* `OBSTACLE_THRESHOLD_M = 0.5`: Distance (in meters). If the closest valid point in an ROI is nearer than this threshold, an avoidance action is triggered. **Tunable.**
* `OBSTACLE_ACTION = "TURN_RIGHT"`: Action taken *only* if the **center** ROI detects an obstacle. Options: `"STOP"`, `"TURN_LEFT"`, `"TURN_RIGHT"`, `"REVERSE"`. Turn actions use counter-spin PWM values defined below.

### Differential Drive PWM Tuning

* **CRITICAL TUNING AREA!** These values **must** be adjusted for your specific rover's motors, ESCs, and gearing. Incorrect values can lead to unexpected behavior or damage.
* `DRIVE_NEUTRAL_PWM = 1500`: PWM value where motors stop. Should match `RCx_TRIM` for the motor channels in ArduPilot if applicable.
* `DRIVE_FORWARD_PWM = 1800`: Target PWM for moving straight forward. Adjust for desired forward speed.
* `DRIVE_REVERSE_PWM = 1150`: Target PWM for moving backward (used in counter-spin turns and reverse actions). Adjust for desired reverse/turn speed. Values further from neutral result in faster movement.

### RC Channel Override Assignment

* `RC_CHAN_LEFT_THROTTLE_IDX = 1`: Index (0-based) in the `RC_CHANNELS_OVERRIDE` message corresponding to the **Left** motor PWM. `1` means MAVLink RC channel **2**.
* `RC_CHAN_RIGHT_THROTTLE_IDX = 0`: Index (0-based) corresponding to the **Right** motor PWM. `0` means MAVLink RC channel **1**.
* **IMPORTANT:** Verify these indices match how your ArduPilot `SERVOx_FUNCTION` settings interpret the RC Override message. Mismatches will cause incorrect motor control (e.g., turning the wrong way).

### Rover Geometry

* `ROVER_WIDTH_M = 0.508`: Physical width of your rover in meters. Used for the geometric turn safety check.
* `SIDE_CLEARANCE_M = 0.07`: Safety buffer (meters). If a side obstacle's detected lateral position (`X` coordinate) is closer to the rover's edge than `(ROVER_WIDTH_M / 2) - SIDE_CLEARANCE_M`, a reverse maneuver is triggered instead of a turn. **Tunable.** Increase if the rover still clips obstacles during turns.

## Usage

1.  **Safety First:** Ensure the rover is safely propped up with wheels off the ground or in an open area clear of valuables and people, especially during initial tuning. **Remove propellers if applicable.**
2.  **Connect Hardware:** Ensure OAK-D, flight controller, and telemetry link are connected to the companion computer.
3.  **Start `mavlink-router`:**
    ```bash
    mavlink-routerd -c /path/to/your/main.conf &
    ```
4.  **Power On:** Power up the rover and flight controller.
5.  **Verify MAVLink:** Check if `mavlink-router` is receiving messages from the flight controller. You can often see this in the `mavlink-routerd` console output or by connecting a GCS through `mavlink-router`.
6.  **ARM the Rover:** Using your RC transmitter or GCS, ARM the flight controller.
7.  **Set Mode:** Switch the flight controller to a mode compatible with RC Overrides (e.g., **Manual**, **Acro**).
8.  **Run the Script:**
    ```bash
    python your_script_name.py
    ```
9.  **Observe:** The script will attempt to connect to `mavlink-router` and the OAK-D, then start printing distance readings and actions taken. Observe the rover's behavior and the console output.
10. **Stop:** Press `Ctrl+C` in the terminal where the script is running to stop it gracefully. The script will attempt to send stop commands (neutral PWM) to the rover before exiting.

## How it Works

### DepthAI Pipeline

1.  **Mono Cameras:** Captures images from the left and right grayscale cameras on the OAK-D.
2.  **StereoDepth Node:** Performs stereo matching between the left and right images to compute a depth map. It applies filters like the confidence threshold.
3.  **SpatialLocationCalculator Node:** Takes the depth map and calculates the 3D coordinates (X, Y, Z) of points within the specified `ROIS`. It uses the configured `SPATIAL_CALC_METHOD` to determine a representative point within each ROI.
    * `X`: Lateral distance (meters, +ve right, -ve left)
    * `Y`: Vertical distance (meters, usually not used in this script)
    * `Z`: Forward distance (meters)
4.  **Output Queue:** The calculated spatial data for each ROI is sent back to the host (companion computer) via an XLinkOut queue.

### MAVLink Communication

1.  **Connection:** The script connects to `mavlink-router` via UDP, specifying the host and port where `mavlink-router` is listening for *incoming* commands for the flight controller.
2.  **Heartbeat Wait:** It waits for a MAVLink HEARTBEAT message from the flight controller (via `mavlink-router`) to confirm the connection is active.
3.  **RC Override:** The script sends `RC_CHANNELS_OVERRIDE` messages. This message allows a companion computer to directly set the PWM output values for specified RC channels, bypassing the normal RC transmitter inputs for those channels. The script populates the array with neutral (`0`) for unused channels and the calculated `target_pwm_left` and `target_pwm_right` for the channels mapped to the motors.

### Obstacle Detection & Logic

1.  **Data Retrieval:** The main loop continuously fetches the latest spatial data from the OAK-D.
2.  **Data Processing:** It extracts the forward distance (`Z`) and lateral distance (`X`) for the closest valid point found within each ROI (`left`, `center`, `right`). Invalid points or points outside the configured depth range (`cfg.depthThresholds`) are ignored.
3.  **Decision Making:**
    * **Default:** Assumes the path is clear and sets target PWMs for forward motion.
    * **Center Obstacle:** If `center_dist < OBSTACLE_THRESHOLD_M`, it performs the action defined by `OBSTACLE_ACTION` (Stop, Turn Left/Right, Reverse) by setting the appropriate `target_pwm_left` and `target_pwm_right`.
    * **Left Obstacle:** If `left_dist < OBSTACLE_THRESHOLD_M` (and center is clear), it triggers a turn-right maneuver (counter-spin: left motor forward, right motor reverse), *unless* the geometric check fails.
    * **Right Obstacle:** If `right_dist < OBSTACLE_THRESHOLD_M` (and center/left are clear), it triggers a turn-left maneuver (counter-spin: left motor reverse, right motor forward), *unless* the geometric check fails.

### Geometric Turn Safety Check

This check prevents the rover from turning *into* an obstacle it's trying to avoid, especially in tight spaces.

* **When Turning Right (due to Left Obstacle):**
    * It checks the obstacle's lateral position (`left_x`). Since `left_x` is negative on the left side, it compares it to the rover's physical left edge coordinate (`-ROVER_HALF_WIDTH_M`) plus a safety margin (`SIDE_CLEARANCE_M`).
    * If `left_x < (-ROVER_HALF_WIDTH_M + SIDE_CLEARANCE_M)`: The obstacle is *too close* laterally (too far left). A right turn would likely cause the rover's left side to hit it. Action changes to **Reverse**.
    * Otherwise: The turn is considered safe. Action proceeds as **Counter-Spin Right**.
* **When Turning Left (due to Right Obstacle):**
    * It checks the obstacle's lateral position (`right_x`). Since `right_x` is positive on the right side, it compares it to the rover's physical right edge coordinate (`ROVER_HALF_WIDTH_M`) minus a safety margin (`SIDE_CLEARANCE_M`).
    * If `right_x > (ROVER_HALF_WIDTH_M - SIDE_CLEARANCE_M)`: The obstacle is *too close* laterally (too far right). A left turn would likely cause the rover's right side to hit it. Action changes to **Reverse**.
    * Otherwise: The turn is considered safe. Action proceeds as **Counter-Spin Left**.

## Troubleshooting / Notes

* **No Movement:**
    * Is the flight controller ARMED and in a suitable MODE (Manual, Acro)?
    * Is `mavlink-router` running and correctly configured to talk to both the flight controller and the script's UDP port (`14551`)?
    * Is the MAVLink connection established (script prints "Heartbeat received!")?
    * Are the `DRIVE_*_PWM` values correct for your hardware? Verify neutral, forward, and reverse using GCS Servo Output or RC Calibration screens first if possible.
    * Are the `RC_CHAN_*_IDX` values correct for your ArduPilot `SERVOx_FUNCTION` setup? Double-check which override channel controls which motor.
* **Turns Wrong Way:** The `RC_CHAN_LEFT_THROTTLE_IDX` and `RC_CHAN_RIGHT_THROTTLE_IDX` are likely swapped relative to your ArduPilot setup, or one motor's `SERVOx_REVERSED` parameter in ArduPilot is incorrect.
* **Hits Obstacles During Turns:** Increase `SIDE_CLEARANCE_M` for a larger safety buffer in the geometric check. The rover's turning radius might be larger than assumed.
* **Poor Obstacle Detection:**
    * Adjust `CONFIDENCE_THRESHOLD`. Lower it if too many points are filtered, increase it if too much noise is present.
    * Adjust `ROIS` to better cover the critical detection areas for your camera angle.
    * Consider environmental factors: very low light, textureless surfaces (blank walls), or highly reflective surfaces can challenge stereo depth perception.
    * Try different `SPATIAL_CALC_METHOD` options (e.g., `AVERAGE`, `MEDIAN`).
* **MAVLink Errors:** Ensure only one process (this script *or* a GCS) is actively sending commands like RC Overrides at a time, unless `mavlink-router` is specifically configured to handle multiple command sources appropriately.

## License

[Specify Your License Here - e.g., MIT, Apache 2.0, GPLv3]

If no license is specified, standard copyright laws apply. Consider adding an open-source license like MIT if you intend for others to use, modify, or distribute this code.
```