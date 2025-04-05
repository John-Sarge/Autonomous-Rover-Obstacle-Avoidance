#!/usr/bin/env python3

"""
Autonomous Rover Obstacle Avoidance Script using OAK-D and MAVLink.

This script uses depth data from an OAK-D camera to detect obstacles
and sends RC Override commands to an ArduPilot Rover (configured for
differential drive / skid steer with specific output functions) via MAVLink
(using mavlink-router) to perform reactive obstacle avoidance.

It includes a basic geometric check to prevent turns that would likely cause
a collision based on the rover's width.
"""

import time
import depthai as dai
import numpy as np
from pymavlink import mavutil
import signal # For handling Ctrl+C (SIGINT)
import traceback # For printing full error details
import cv2 # OpenCV for image processing and overlay

# =============================================================================
# --- Configuration Constants (ADJUST THESE CAREFULLY!) ---
# =============================================================================

# --- MAVLink Connection Settings ---
MAVLINK_ROUTER_HOST = '127.0.0.1' # IP address of where mavlink-router is running (usually localhost)
MAVLINK_ROUTER_PORT = 14551      # UDP Port mavlink-router listens on for commands FROM this script
                                 # This MUST match a UdpEndpoint Port with Mode=Server in main.conf
MAVLINK_SYSTEM_ID = 1            # Target System ID of your Pixhawk (usually 1)
MAVLINK_COMPONENT_ID = 1         # Target Component ID of your Pixhawk (usually 1)
GCS_SOURCE_SYSTEM = 255          # System ID for this script (255 is often used for GCS/Companion Computer)

# --- OAK-D / DepthAI Settings ---
# Resolution for the mono cameras used for depth calculation. Lower = faster but potentially less detail.
DEPTH_RESOLUTION = dai.MonoCameraProperties.SensorResolution.THE_400_P # Options: THE_400_P, THE_720_P, THE_800_P
# Target frame rate for depth processing. Higher = more up-to-date info but more Pi CPU usage.
DEPTH_RATE_HZ = 15
# Minimum confidence score (0-255) for a depth pixel to be considered valid. Higher values filter more aggressively.
# TUNABLE: Adjust based on environment lighting/texture and noise levels (e.g., 180-240).
CONFIDENCE_THRESHOLD = 220 # Your value

# --- Regions of Interest (ROIs) for Obstacle Detection ---
# Define areas in the camera's normalized view (0.0 to 1.0) to check for obstacles.
# Format: ("Name", dai.Rect(Point2f(Xmin, Ymin), Point2f(Xmax, Ymax)))
# (0,0) is top-left, (1,1) is bottom-right.
# Y values closer to 0 look higher up; Y values closer to 1 look lower down.
# X values closer to 0 are left; X values closer to 1 are right.
# **TUNABLE**: Adjust these rectangles based on camera angle, field of view, rover width,
#             and where you most critically need to detect obstacles. Visualization helps!
ROIS = [
    # Name      TopLeft(X,Y)        BottomRight(X,Y)
    ("center", dai.Rect(dai.Point2f(0.4, 0.5), dai.Point2f(0.6, 0.9))), # Center 20% width, lower half
    ("left",   dai.Rect(dai.Point2f(0.1, 0.5), dai.Point2f(0.4, 0.9))), # Left zone (X: 10%-40%)
    ("right",  dai.Rect(dai.Point2f(0.6, 0.5), dai.Point2f(0.75, 0.9))),# Right zone (X: 60%-75%) - Note: Still slightly asymmetric width vs left ROI
]
# Method used by OAK-D's SpatialLocationCalculator to determine the representative distance within an ROI.
# MIN: Returns the closest valid point found in the ROI (sensitive to noise).
# AVERAGE: Averages valid points in ROI (smoother but might miss small obstacles).
# MEDIAN: Median distance of valid points (robust to outliers).
# MODE: Most frequent distance value.
SPATIAL_CALC_METHOD = dai.SpatialLocationCalculatorAlgorithm.MIN # MIN seemed necessary in your tests

# --- Obstacle Avoidance Logic Tuning ---
# Distance threshold (meters). If closest point in an ROI is below this, trigger a reaction.
OBSTACLE_THRESHOLD_M = 0.5       # TUNABLE: Your preferred value for office. Needs careful balance with speed.
# Action to take ONLY if the CENTER ROI detects an obstacle below threshold.
# Options: "STOP", "TURN_RIGHT", "TURN_LEFT", "REVERSE"
# Note: TURN actions use counter-spin (defined by PWMs below).
OBSTACLE_ACTION = "TURN_RIGHT"   # TUNABLE: Set preferred center-obstacle reaction.

# --- Differential Drive PWM Tuning ---
# ** CRITICAL TUNING AREA ** - Adjust for your specific motors/ESCs!
# PWM value where motors stop (verify with ESC calibration). Should match RCx_TRIM.
DRIVE_NEUTRAL_PWM = 1500
# Target PWM for moving straight forward. Start low and increase until desired speed is reached.
DRIVE_FORWARD_PWM = 1800         # TUNABLE: Adjust for speed (e.g., 1550=slow, 1700=medium, 1850=fast?)
# Target PWM for moving backward (used for counter-spin turns and backup). Adjust for reverse/turn speed.
DRIVE_REVERSE_PWM = 1150         # TUNABLE: Further from neutral = faster (e.g., 1450=slow, 1350=medium, 1250=fast?)

# --- RC Channel Override Assignment (Your empirically found working setup) ---
# Maps desired Left/Right motor control to specific RC Override channels sent via MAVLink.
# ** IMPORTANT **: Ensure these indices match how your specific ArduPilot setup interprets overrides
#                  for SERVO1_FUNCTION=73 and SERVO3_FUNCTION=74.
RC_CHAN_LEFT_THROTTLE_IDX = 1  # MAVLink Override CH2 (Index 1) controls Left Motor?
RC_CHAN_RIGHT_THROTTLE_IDX = 0 # MAVLink Override CH1 (Index 0) controls Right Motor?

# --- Rover Geometry ---
# Used for the geometric check to prevent unsafe turns.
ROVER_WIDTH_M = 0.508            # Rover width in meters (20 inches)
ROVER_HALF_WIDTH_M = ROVER_WIDTH_M / 2.0 # Approx 0.254m
# Safety buffer (meters). If obstacle's lateral position (X) is inside this distance
# from the rover's physical edge, the turn is aborted (and backup initiated).
SIDE_CLEARANCE_M = 0.07         # TUNABLE: Increase (e.g., 0.1, 0.15) if still hitting sides during turns.

# GStreamer Video Output Settings
GSTREAMER_PIPELINE = (
    "appsrc ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! "
    "udpsink host=127.0.0.1 port=5600"  # Update host and port for QGroundControl
)

# =============================================================================
# --- Global Variables & Signal Handling ---
# =============================================================================
master = None # Holds the MAVLink connection object
running = True # Flag to control the main loop, set to False by signal handler

def handle_exit(signum, frame):
    """Signal handler to set the running flag to False on Ctrl+C or kill."""
    global running
    print("Exit signal received. Stopping rover...")
    running = False

# Register signal handlers for graceful shutdown
signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

# =============================================================================
# --- Helper Functions ---
# =============================================================================
def connect_mavlink():
    """Establishes connection to the Pixhawk via mavlink-router."""
    # Use 'udpout' to specify we are SENDING TO this address/port
    connection_string = f'udpout:{MAVLINK_ROUTER_HOST}:{MAVLINK_ROUTER_PORT}'
    print(f"Connecting to MAVLink router at {connection_string}")
    try:
        # Create the connection object
        mav_conn = mavutil.mavlink_connection(connection_string, source_system=GCS_SOURCE_SYSTEM)
        # Wait for the first heartbeat message from the Pixhawk to confirm connection
        print("Waiting for heartbeat..."); mav_conn.wait_heartbeat(timeout=10)
        print(f"Heartbeat received! (SysID {mav_conn.target_system}, CompID {mav_conn.target_component})")
        return mav_conn
    except Exception as e:
        # Handle connection errors
        print(f"MAVLink connection error: {e}")
        return None

def create_depthai_pipeline():
    """Creates the DepthAI pipeline: Mono cams -> Stereo -> SpatialCalculator -> RGB."""
    print("Creating DepthAI pipeline...")
    pipeline = dai.Pipeline()

    # 1. Define Nodes
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    spatialCalc = pipeline.create(dai.node.SpatialLocationCalculator)
    rgbCamera = pipeline.create(dai.node.ColorCamera)
    xoutSpatialCalc = pipeline.create(dai.node.XLinkOut) # Output for spatial data
    xoutRgb = pipeline.create(dai.node.XLinkOut) # Output for RGB stream
    configIn = pipeline.create(dai.node.XLinkIn)         # Input for runtime config

    configIn.setStreamName("spatialConfig")       # Name for the config input queue
    xoutSpatialCalc.setStreamName("spatialData")  # Name for the results output queue
    xoutRgb.setStreamName("rgb")                  # Name for the RGB output queue

    # 2. Configure Nodes
    # Set resolution and source for mono cameras
    monoLeft.setResolution(DEPTH_RESOLUTION); monoLeft.setCamera("left"); monoLeft.setFps(DEPTH_RATE_HZ)
    monoRight.setResolution(DEPTH_RESOLUTION); monoRight.setCamera("right"); monoRight.setFps(DEPTH_RATE_HZ)

    # Configure stereo depth node
    stereo.setConfidenceThreshold(CONFIDENCE_THRESHOLD) # Filter depth pixels below this confidence
    stereo.setLeftRightCheck(True)       # Better handling of occlusions
    stereo.setSubpixel(False)          # Faster, less precise depth
    stereo.setExtendedDisparity(False) # Use for closer distances, disables subpixel
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY) # Preset for accuracy
    stereo.setRectifyEdgeFillColor(0)  # Fill areas with no depth info with black

    # Configure RGB camera
    rgbCamera.setBoardSocket(dai.CameraBoardSocket.RGB)
    rgbCamera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgbCamera.setFps(DEPTH_RATE_HZ)

    # 3. Prepare Spatial Calculator configuration object (to be sent after pipeline start)
    spatial_config_object = dai.SpatialLocationCalculatorConfig()
    print("Preparing ROIs for Spatial Calculator:")
    cfg_data_list = []
    # Create configuration data for each defined ROI
    for name, roi_rect in ROIS:
        cfg = dai.SpatialLocationCalculatorConfigData()
        cfg.roi = roi_rect                      # Set the rectangular region
        cfg.calculationAlgorithm = SPATIAL_CALC_METHOD # Set how to calculate distance
        # Set min/max distance limits (in mm) for points considered by the calculator
        cfg.depthThresholds.lowerThreshold = 300 # Ignore points closer than 0.3m
        cfg.depthThresholds.upperThreshold = 5000 # Ignore points further than 5.0m
        cfg_data_list.append(cfg)
        # Print configured ROI details
        print(f"- ROI '{name}': Algorithm={SPATIAL_CALC_METHOD}, Rect={roi_rect.topLeft().x:.2f},{roi_rect.topLeft().y:.2f} -> {roi_rect.bottomRight().x:.2f},{roi_rect.bottomRight().y:.2f}")
    # Add all the ROI configurations to the main config object
    spatial_config_object.setROIs(cfg_data_list)

    # 4. Link Nodes Together
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    stereo.depth.link(spatialCalc.inputDepth)      # Feed depth map to spatial calculator
    spatialCalc.out.link(xoutSpatialCalc.input)    # Send results to output queue
    configIn.out.link(spatialCalc.inputConfig)     # Link input queue to config input
    rgbCamera.video.link(xoutRgb.input)            # Link RGB output

    print("Pipeline created.")
    # Return the pipeline structure AND the prepared config object
    return pipeline, spatial_config_object

def start_gstreamer_pipeline(pipeline_desc):
    """Starts a GStreamer pipeline for video streaming."""
    return subprocess.Popen(pipeline_desc, shell=True, stdin=subprocess.PIPE, bufsize=0)
def send_diff_drive_override(mav_connection, left_pwm, right_pwm):
    """Sends RC_CHANNELS_OVERRIDE for differential drive based on discovered channel mapping."""
    if not mav_connection: return # Do nothing if connection lost

    # Clamp PWM values to the standard 1000-2000 range
    left_pwm = max(1000, min(2000, int(left_pwm)))
    right_pwm = max(1000, min(2000, int(right_pwm)))

    # Print the command being sent for real-time debugging
    print(f"DEBUG: Sending Diff Drive Override: CH{RC_CHAN_LEFT_THROTTLE_IDX + 1}={left_pwm}, CH{RC_CHAN_RIGHT_THROTTLE_IDX + 1}={right_pwm}")

    # Create the MAVLink RC Channels Override message data structure
    rc_override_values = [0] * 18 # Initialize all 18 channels to 0 (means "no override")
    # Set the PWM values for the specific channels controlling the motors
    rc_override_values[RC_CHAN_LEFT_THROTTLE_IDX] = left_pwm  # CH2 controls Left Motor
    rc_override_values[RC_CHAN_RIGHT_THROTTLE_IDX] = right_pwm # CH1 controls Right Motor

    try:
        # Send the command. Use slicing '[:8]' to send only the first 8 channels,
        # which seems necessary for compatibility with some Pymavlink/ArduPilot versions.
        mav_connection.mav.rc_channels_override_send(
            mav_connection.target_system,       # Target Pixhawk system ID
            mav_connection.target_component,    # Target Pixhawk component ID
            *rc_override_values[:8]             # Unpack first 8 channel values as arguments
        )
    except Exception as e:
        # Catch and print errors during MAVLink sending
        print(f"Error sending RC override: {e}")

def stop_rover(mav_connection):
    """Sends neutral RC overrides repeatedly to ensure the rover stops."""
    print("Sending stop command (neutral throttles)...")
    if not mav_connection: return # Check connection
    # Send neutral command multiple times in case of packet loss
    for _ in range(10):
        send_diff_drive_override(mav_connection, DRIVE_NEUTRAL_PWM, DRIVE_NEUTRAL_PWM)
        time.sleep(0.05) # Small delay between commands
    print("Stop command sent.")

# =============================================================================
# --- Main Execution ---
# =============================================================================
if __name__ == "__main__":
    # Initialize MAVLink connection
    master = connect_mavlink()
    if not master: exit(1) # Exit if cannot connect

    # Initialize DepthAI pipeline
    pipeline, spatial_config_to_send = create_depthai_pipeline()

    try:
        # Connect to OAK-D camera and start the pipeline
        with dai.Device(pipeline) as device:
            print("DepthAI device found.")
            # Get the output queue for spatial data results
            spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
            rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            # Get the input queue for sending configuration
            configQueue = device.getInputQueue("spatialConfig")

            # Start GStreamer pipeline for streaming
            gstreamer_process = start_gstreamer_pipeline(GSTREAMER_PIPELINE)

            # Send the prepared ROI configuration to the spatial calculator node on the OAK-D
            print("Sending spatial calculator config to device..."); configQueue.send(spatial_config_to_send); print("Config sent.")

            # Print initial status message
            print("\n" + "="*30)
            print("Rover script running (Differential Drive Logic + Geo Check/Backup). Ctrl+C to exit.")
            print(f"Obstacle Threshold: {OBSTACLE_THRESHOLD_M}m")
            print(f"Forward PWM: {DRIVE_FORWARD_PWM}")
            print(f"Reverse PWM: {DRIVE_REVERSE_PWM}")
            print(f"Rover Half Width: {ROVER_HALF_WIDTH_M:.3f}m")
            print(f"Side Clearance: {SIDE_CLEARANCE_M}m")
            print("Ensure Pixhawk is ARMED and in MANUAL or ACRO mode (Verify PILOT_STEER_TYPE=1).") # Mode reminder
            print("="*30 + "\n")

            # Main control loop
            while running: # Loop continues until Ctrl+C is pressed (sets running=False)
                # Get the latest spatial calculation results from OAK-D (waits for new data)
                inSpatialData = spatialCalcQueue.get();
                if inSpatialData is None: time.sleep(0.05); continue # Skip if queue empty

                spatialData = inSpatialData.getSpatialLocations(); # Extract the list of results

                # Process results into a dictionary: {'roi_name': (dist_m, x_m)}
                obstacle_data = {}
                # Ensure the number of results matches the number of ROIs configured
                if len(spatialData) == len(ROIS):
                    for i, d in enumerate(spatialData):
                        roi_name = ROIS[i][0] # Get the name ('center', 'left', 'right')
                        dist_m = 999          # Default distance if invalid (effectively "clear")
                        x_m = 0               # Default lateral position
                        coords = d.spatialCoordinates # Get X,Y,Z coordinates object
                        # Check if depth reading is valid (Z>0) and within specified range
                        if coords.z > 0 and d.config.depthThresholds.lowerThreshold <= coords.z <= d.config.depthThresholds.upperThreshold:
                            dist_m = coords.z / 1000.0 # Forward distance in meters
                            x_m = coords.x / 1000.0    # Lateral distance in meters (+ve is right, -ve is left)
                        obstacle_data[roi_name] = (dist_m, x_m) # Store tuple
                else:
                    # If data mismatch, wait briefly and try next cycle
                    time.sleep(0.1); continue
                # Extract specific distances and lateral positions, defaulting to 'clear' values
                center_dist, _ = obstacle_data.get("center", (999, 0))
                left_dist, left_x = obstacle_data.get("left", (999, 0))
                right_dist, right_x = obstacle_data.get("right", (999, 0))
                # Log current sensor readings
                print(f"Distances (m): L={left_dist:.2f}(X:{left_x:.2f}) C={center_dist:.2f} R={right_dist:.2f}(X:{right_x:.2f})")

                # --- Reactive Logic (Differential Drive with Counter-Spin & Geometry Check/Backup) ---
                # Default action assumes path is clear: drive both motors forward
                target_pwm_left = DRIVE_FORWARD_PWM
                target_pwm_right = DRIVE_FORWARD_PWM
                action_taken = "Path Clear -> Moving Forward"

                # Check for obstacles, prioritizing center, then left, then right
                if center_dist < OBSTACLE_THRESHOLD_M:
                    # Center obstacle detected - perform the primary configured action
                    action_taken = f"Obstacle Center ({center_dist:.2f}m)"
                    if OBSTACLE_ACTION == "STOP":
                        target_pwm_left, target_pwm_right = DRIVE_NEUTRAL_PWM, DRIVE_NEUTRAL_PWM; action_taken += " -> Stopping"
                    elif OBSTACLE_ACTION == "TURN_RIGHT":
                        target_pwm_left, target_pwm_right = DRIVE_FORWARD_PWM, DRIVE_REVERSE_PWM; action_taken += " -> Turning Right (in place)"
                    elif OBSTACLE_ACTION == "TURN_LEFT":
                        target_pwm_left, target_pwm_right = DRIVE_REVERSE_PWM, DRIVE_FORWARD_PWM; action_taken += " -> Turning Left (in place)"
                    elif OBSTACLE_ACTION == "REVERSE":
                        target_pwm_left, target_pwm_right = DRIVE_REVERSE_PWM, DRIVE_REVERSE_PWM; action_taken += " -> Reversing"

                elif left_dist < OBSTACLE_THRESHOLD_M:
                    # Left obstacle detected
                    action_taken = f"Obstacle Left ({left_dist:.2f}m)"
                    # ** Geometry Check **
                    # Check if the detected point's X coordinate is already too close to the rover's physical left side.
                    # left_x is negative for left side. -ROVER_HALF_WIDTH_M is the coordinate of the left edge.
                    # Add a clearance margin. If left_x is less (more negative) than this limit, turn is unsafe.
                    if left_x < (-ROVER_HALF_WIDTH_M + SIDE_CLEARANCE_M):
                        # Unsafe Turn -> Command REVERSE
                        action_taken += f" (X={left_x:.2f}m TOO CLOSE!) -> Reversing"
                        target_pwm_left = DRIVE_REVERSE_PWM
                        target_pwm_right = DRIVE_REVERSE_PWM
                    else:
                        # Safe to turn right -> Command Counter-Spin Right
                        action_taken += f" (X={left_x:.2f}m OK) -> Counter-Spinning Right"
                        target_pwm_left = DRIVE_FORWARD_PWM
                        target_pwm_right = DRIVE_REVERSE_PWM

                elif right_dist < OBSTACLE_THRESHOLD_M:
                    # Right obstacle detected
                    action_taken = f"Obstacle Right ({right_dist:.2f}m)"
                    # ** Geometry Check **
                    # Check if the detected point's X coordinate is already too close to the rover's physical right side.
                    # right_x is positive for right side. ROVER_HALF_WIDTH_M is the coordinate of the right edge.
                    # Subtract a clearance margin. If right_x is greater than this limit, turn is unsafe.
                    if right_x > (ROVER_HALF_WIDTH_M - SIDE_CLEARANCE_M):
                        # Unsafe Turn -> Command REVERSE
                        action_taken += f" (X={right_x:.2f}m TOO CLOSE!) -> Reversing"
                        target_pwm_left = DRIVE_REVERSE_PWM
                        target_pwm_right = DRIVE_REVERSE_PWM
                    else:
                        # Safe to turn left -> Command Counter-Spin Left
                        action_taken += f" (X={right_x:.2f}m OK) -> Counter-Spinning Left"
                        target_pwm_left = DRIVE_REVERSE_PWM
                        target_pwm_right = DRIVE_FORWARD_PWM
                else:
                    # Path Clear - action_taken already set to Forward
                    pass

                # Print the action decided for this loop cycle
                print(action_taken)

                # --- Send Commands to Pixhawk ---
                if running: # Check if exit signal was received
                    send_diff_drive_override(master, target_pwm_left, target_pwm_right)
                else:
                    # If exit signal received, stop rover and break loop
                    stop_rover(master)
                    break

                # Get the latest RGB frame
                rgbFrame = rgbQueue.tryGet()
                if rgbFrame is not None:
                    # Convert the frame to a format OpenCV can use
                    frame = rgbFrame.getCvFrame()
                    # Overlay obstacle data on image
                    for name, (dist_m, x_m) in obstacle_data.items():
                        color = (0, 255, 0) if dist_m >= OBSTACLE_THRESHOLD_M else (0, 0, 255)
                        cv2.putText(frame, f"{name}: {dist_m:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    # Stream frame with overlay using GStreamer
                    gstreamer_process.stdin.write(frame.tobytes())

                # --- Loop Delay ---
                # Controls the rate of the control loop (~10Hz).
                time.sleep(0.1)

    # Handle errors gracefully
    except Exception as e:
        print(f"An error occurred: {e}")
        traceback.print_exc() # Print detailed error information
    # Cleanup sequence runs regardless of whether loop exited normally or via error/Ctrl+C
    finally:
        print("Exiting script.")
        if master: # Check if mavlink connection exists
            stop_rover(master) # Ensure motors are stopped
            master.close()     # Close the connection
            print("MAVLink connection closed.")
        if gstreamer_process:
            gstreamer_process.terminate()
        print("Script finished cleanly.")
