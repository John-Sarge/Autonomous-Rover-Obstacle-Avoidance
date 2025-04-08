#!/usr/bin/env python3
"""
Autonomous Rover Obstacle Avoidance Script using OAK-D and MAVLink.

Based on original code by John Seargeant.
Modifications:
- Added neutral pause before turns to reduce forward spurt.
- Added local OpenCV preview window with ROI and status overlays.
"""

import time
import depthai as dai
import numpy as np
from pymavlink import mavutil
import signal
import traceback
import cv2 # *** ADDED IMPORT for OpenCV ***

# =============================================================================
# --- Configuration Constants (ADJUST THESE CAREFULLY!) ---
# =============================================================================

# --- MAVLink Connection Settings ---
MAVLINK_ROUTER_HOST = '127.0.0.1'
MAVLINK_ROUTER_PORT = 14551
MAVLINK_SYSTEM_ID = 1
MAVLINK_COMPONENT_ID = 1
GCS_SOURCE_SYSTEM = 255

# --- OAK-D / DepthAI Settings ---
DEPTH_RESOLUTION = dai.MonoCameraProperties.SensorResolution.THE_400_P
DEPTH_RATE_HZ = 15
CONFIDENCE_THRESHOLD = 200
# --- NEW: Preview Size (MUST MATCH rgbCamera.setPreviewSize) ---
PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 360

# --- Regions of Interest (ROIs) for Obstacle Detection ---
# Define areas in the camera's normalized view (0.0 to 1.0) to check for obstacles.
# Format: ("Name", dai.Rect(Point2f(Xmin, Ymin), Point2f(Xmax, Ymax)))
# (0,0) is top-left, (1,1) is bottom-right.
# Y values closer to 0 look higher up; Y values closer to 1 look lower down.
# X values closer to 0 are left; X values closer to 1 are right.
# **TUNABLE**: Adjust these rectangles based on camera angle, field of view, rover width,
#             and where you most critically need to detect obstacles. Visualization helps!
ROIS = [
    # Name      TopLeft(X,Y)         BottomRight(X,Y)
    ("center", dai.Rect(dai.Point2f(0.4, 0.5), dai.Point2f(0.6, 0.9))), # Center 20% width, lower half
    ("left",   dai.Rect(dai.Point2f(0.1, 0.5), dai.Point2f(0.4, 0.9))), # Left zone (X: 10%-40%)
    ("right",  dai.Rect(dai.Point2f(0.6, 0.5), dai.Point2f(0.75, 0.9))),# Right zone (X: 60%-75%) - Note: Still slightly asymmetric width vs left ROI
]
SPATIAL_CALC_METHOD = dai.SpatialLocationCalculatorAlgorithm.MIN

# --- Obstacle Avoidance Logic Tuning ---
OBSTACLE_THRESHOLD_M = 0.55
OBSTACLE_ACTION = "TURN_RIGHT"

# --- Differential Drive PWM Tuning ---
DRIVE_NEUTRAL_PWM = 1500
DRIVE_FORWARD_PWM = 1750
DRIVE_REVERSE_PWM = 1200

# --- Turn Initiation Pause ---
TURN_INITIATION_PAUSE_S = 0.0

# --- RC Channel Override Assignment ---
RC_CHAN_LEFT_THROTTLE_IDX = 1
RC_CHAN_RIGHT_THROTTLE_IDX = 0

# --- Rover Geometry ---
ROVER_WIDTH_M = 0.508
ROVER_HALF_WIDTH_M = ROVER_WIDTH_M / 2.0
SIDE_CLEARANCE_M = 0.075

# --- NEW: Preview Window Configuration ---
PREVIEW_WINDOW_NAME = "OAK-D Preview"
OVERLAY_COLOR_CLEAR = (0, 255, 0) # Green
OVERLAY_COLOR_OBSTACLE = (0, 0, 255) # Red
OVERLAY_COLOR_TEXT = (255, 255, 0) # Cyan/Yellow

# =============================================================================
# --- Global Variables & Signal Handling ---
# =============================================================================
master = None
running = True
previous_pwm_left = DRIVE_NEUTRAL_PWM
previous_pwm_right = DRIVE_NEUTRAL_PWM

def handle_exit(signum, frame):
    global running
    print("Exit signal received. Stopping rover...")
    running = False

signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

# =============================================================================
# --- Helper Functions ---
# =============================================================================
def connect_mavlink():
    connection_string = f'udpout:{MAVLINK_ROUTER_HOST}:{MAVLINK_ROUTER_PORT}'
    print(f"Connecting to MAVLink router at {connection_string}")
    try:
        mav_conn = mavutil.mavlink_connection(connection_string, source_system=GCS_SOURCE_SYSTEM)
        print("Waiting for heartbeat..."); mav_conn.wait_heartbeat(timeout=10)
        print(f"Heartbeat received! (SysID {mav_conn.target_system}, CompID {mav_conn.target_component})")
        return mav_conn
    except Exception as e:
        print(f"MAVLink connection error: {e}")
        return None

# --- MODIFIED Function to include RGB Camera output ---
def create_depthai_pipeline():
    print("Creating DepthAI pipeline...")
    pipeline = dai.Pipeline()

    # Nodes
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    spatialCalc = pipeline.create(dai.node.SpatialLocationCalculator)
    configIn = pipeline.create(dai.node.XLinkIn)
    xoutSpatialCalc = pipeline.create(dai.node.XLinkOut)

    # --- NEW: RGB Camera Nodes ---
    rgbCamera = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)

    # Stream Names
    configIn.setStreamName("spatialConfig")
    xoutSpatialCalc.setStreamName("spatialData")
    xoutRgb.setStreamName("rgb") # NEW: RGB output stream name

    # Properties - Mono Cams
    monoLeft.setResolution(DEPTH_RESOLUTION); monoLeft.setCamera("left"); monoLeft.setFps(DEPTH_RATE_HZ)
    monoRight.setResolution(DEPTH_RESOLUTION); monoRight.setCamera("right"); monoRight.setFps(DEPTH_RATE_HZ)

    # Properties - Stereo Depth
    stereo.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)
    stereo.setExtendedDisparity(False)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setRectifyEdgeFillColor(0)

    # --- NEW: Properties - RGB Camera ---
    rgbCamera.setBoardSocket(dai.CameraBoardSocket.CAM_A) # Use CAM_A
    rgbCamera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P) # Sensor resolution
    rgbCamera.setFps(DEPTH_RATE_HZ)
    rgbCamera.setPreviewSize(PREVIEW_WIDTH, PREVIEW_HEIGHT) # Output preview size
    rgbCamera.setInterleaved(False)
    rgbCamera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR) # BGR for OpenCV

    # Spatial Config Object (remains the same)
    spatial_config_object = dai.SpatialLocationCalculatorConfig()
    print("Preparing ROIs for Spatial Calculator:")
    cfg_data_list = []
    for name, roi_rect in ROIS:
        cfg = dai.SpatialLocationCalculatorConfigData()
        cfg.roi = roi_rect
        cfg.calculationAlgorithm = SPATIAL_CALC_METHOD
        cfg.depthThresholds.lowerThreshold = 300
        cfg.depthThresholds.upperThreshold = 5000
        cfg_data_list.append(cfg)
        print(f"- ROI '{name}': Algorithm={SPATIAL_CALC_METHOD}, Rect={roi_rect.topLeft().x:.2f},{roi_rect.topLeft().y:.2f} -> {roi_rect.bottomRight().x:.2f},{roi_rect.bottomRight().y:.2f}")
    spatial_config_object.setROIs(cfg_data_list)

    # Linking
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    stereo.depth.link(spatialCalc.inputDepth)
    spatialCalc.out.link(xoutSpatialCalc.input)
    configIn.out.link(spatialCalc.inputConfig)
    rgbCamera.preview.link(xoutRgb.input) # NEW: Link preview output to XLinkOut

    print("Pipeline created.")
    return pipeline, spatial_config_object

# (send_diff_drive_override remains the same as previous version with pause logic)
def send_diff_drive_override(mav_connection, left_pwm, right_pwm):
    global previous_pwm_left, previous_pwm_right
    if not mav_connection: return
    target_left_pwm = max(1000, min(2000, int(left_pwm)))
    target_right_pwm = max(1000, min(2000, int(right_pwm)))
    was_moving_forward = (previous_pwm_left == DRIVE_FORWARD_PWM and previous_pwm_right == DRIVE_FORWARD_PWM)
    is_starting_counter_spin = ( (target_left_pwm == DRIVE_FORWARD_PWM and target_right_pwm == DRIVE_REVERSE_PWM) or \
                                 (target_left_pwm == DRIVE_REVERSE_PWM and target_right_pwm == DRIVE_FORWARD_PWM) )
    pause_needed = was_moving_forward and is_starting_counter_spin and TURN_INITIATION_PAUSE_S > 0
    if pause_needed:
        print("DEBUG: Turn detected from forward. Applying neutral pause.")
        temp_rc_override = [0] * 18
        temp_rc_override[RC_CHAN_LEFT_THROTTLE_IDX] = DRIVE_NEUTRAL_PWM
        temp_rc_override[RC_CHAN_RIGHT_THROTTLE_IDX] = DRIVE_NEUTRAL_PWM
        try:
            mav_connection.mav.rc_channels_override_send(mav_connection.target_system, mav_connection.target_component, *temp_rc_override[:8])
            previous_pwm_left = DRIVE_NEUTRAL_PWM
            previous_pwm_right = DRIVE_NEUTRAL_PWM
        except Exception as e:
            print(f"Error sending neutral RC override: {e}")
            pause_needed = False
        else:
            time.sleep(TURN_INITIATION_PAUSE_S)
            print(f"DEBUG: Pause finished. Sending turn command: L={target_left_pwm}, R={target_right_pwm}")
    # print(f"DEBUG: Sending Diff Drive Override: CH{RC_CHAN_LEFT_THROTTLE_IDX + 1}={target_left_pwm}, CH{RC_CHAN_RIGHT_THROTTLE_IDX + 1}={target_right_pwm}")
    rc_override_values = [0] * 18
    rc_override_values[RC_CHAN_LEFT_THROTTLE_IDX] = target_left_pwm
    rc_override_values[RC_CHAN_RIGHT_THROTTLE_IDX] = target_right_pwm
    try:
        mav_connection.mav.rc_channels_override_send(mav_connection.target_system, mav_connection.target_component, *rc_override_values[:8])
        # Update state based on the command actually sent
        previous_pwm_left = target_left_pwm
        previous_pwm_right = target_right_pwm
    except Exception as e:
        print(f"Error sending RC override: {e}")

# (stop_rover remains the same as previous version)
def stop_rover(mav_connection):
    global previous_pwm_left, previous_pwm_right
    print("Sending stop command (neutral throttles)...")
    target_left_pwm = DRIVE_NEUTRAL_PWM
    target_right_pwm = DRIVE_NEUTRAL_PWM
    if not mav_connection: return
    for _ in range(10):
        rc_override_values = [0] * 18
        rc_override_values[RC_CHAN_LEFT_THROTTLE_IDX] = target_left_pwm
        rc_override_values[RC_CHAN_RIGHT_THROTTLE_IDX] = target_right_pwm
        try:
             mav_connection.mav.rc_channels_override_send(mav_connection.target_system, mav_connection.target_component, *rc_override_values[:8])
        except Exception as e: print(f"Error sending stop RC override: {e}")
        time.sleep(0.05)
    previous_pwm_left = target_left_pwm
    previous_pwm_right = target_right_pwm
    print("Stop command sent.")

# =============================================================================
# --- Main Execution ---
# =============================================================================
if __name__ == "__main__":
    master = connect_mavlink()
    if not master: exit(1)

    pipeline, spatial_config_to_send = create_depthai_pipeline()

    try:
        with dai.Device(pipeline) as device:
            print("DepthAI device found.")
            spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
            configQueue = device.getInputQueue("spatialConfig")
            # --- NEW: Get RGB Queue ---
            rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

            print("Sending spatial calculator config to device..."); configQueue.send(spatial_config_to_send); print("Config sent.")

            print("\n" + "="*30)
            print("Rover script running (Turn Pause + OpenCV Preview). Ctrl+C or 'q' in window to exit.") # Updated title
            # ... print other constants ...
            print("="*30 + "\n")

            # Initialize variables before loop
            frame = None
            obstacle_data = {} # Store latest valid obstacle data
            action_taken = "Initializing"
            target_pwm_left = DRIVE_NEUTRAL_PWM # Start stopped
            target_pwm_right = DRIVE_NEUTRAL_PWM

            while running:
                # --- Get Data ---
                inSpatialData = spatialCalcQueue.tryGet() # Use tryGet for non-blocking
                inRgb = rgbQueue.tryGet() # Use tryGet for non-blocking

                # --- Update Frame if available ---
                if inRgb is not None:
                    frame = inRgb.getCvFrame()

                # --- Process Spatial Data & Decide Action (if available) ---
                if inSpatialData is not None:
                    spatialData = inSpatialData.getSpatialLocations()
                    current_obstacle_data = {} # Process this cycle's data
                    if len(spatialData) == len(ROIS):
                        for i, d in enumerate(spatialData):
                            roi_name = ROIS[i][0]
                            dist_m = 999; x_m = 0
                            coords = d.spatialCoordinates
                            if coords.z > 0 and d.config.depthThresholds.lowerThreshold <= coords.z <= d.config.depthThresholds.upperThreshold:
                                dist_m = coords.z / 1000.0
                                x_m = coords.x / 1000.0
                            current_obstacle_data[roi_name] = (dist_m, x_m)
                        obstacle_data = current_obstacle_data # Update shared data

                        center_dist, _ = obstacle_data.get("center", (999, 0))
                        left_dist, left_x = obstacle_data.get("left", (999, 0))
                        right_dist, right_x = obstacle_data.get("right", (999, 0))
                        print(f"Distances (m): L={left_dist:.2f}(X:{left_x:.2f}) C={center_dist:.2f} R={right_dist:.2f}(X:{right_x:.2f})")

                        # --- Reactive Logic (Determine target PWMs based on current_obstacle_data) ---
                        current_target_pwm_left = DRIVE_FORWARD_PWM
                        current_target_pwm_right = DRIVE_FORWARD_PWM
                        action_taken = "Path Clear -> Moving Forward"

                        if center_dist < OBSTACLE_THRESHOLD_M:
                            action_taken = f"Obstacle Center ({center_dist:.2f}m)"
                            if OBSTACLE_ACTION == "STOP":
                                current_target_pwm_left, current_target_pwm_right = DRIVE_NEUTRAL_PWM, DRIVE_NEUTRAL_PWM; action_taken += " -> Stopping"
                            elif OBSTACLE_ACTION == "TURN_RIGHT":
                                current_target_pwm_left, current_target_pwm_right = DRIVE_FORWARD_PWM, DRIVE_REVERSE_PWM; action_taken += " -> Turning Right (in place)"
                            elif OBSTACLE_ACTION == "TURN_LEFT":
                                current_target_pwm_left, current_target_pwm_right = DRIVE_REVERSE_PWM, DRIVE_FORWARD_PWM; action_taken += " -> Turning Left (in place)"
                            elif OBSTACLE_ACTION == "REVERSE":
                                current_target_pwm_left, current_target_pwm_right = DRIVE_REVERSE_PWM, DRIVE_REVERSE_PWM; action_taken += " -> Reversing"
                        elif left_dist < OBSTACLE_THRESHOLD_M:
                            action_taken = f"Obstacle Left ({left_dist:.2f}m)"
                            if left_x < (-ROVER_HALF_WIDTH_M + SIDE_CLEARANCE_M):
                                action_taken += f" (X={left_x:.2f}m TOO CLOSE!) -> Reversing"
                                current_target_pwm_left = DRIVE_REVERSE_PWM
                                current_target_pwm_right = DRIVE_REVERSE_PWM
                            else:
                                action_taken += f" (X={left_x:.2f}m OK) -> Counter-Spinning Right"
                                current_target_pwm_left = DRIVE_FORWARD_PWM
                                current_target_pwm_right = DRIVE_REVERSE_PWM
                        elif right_dist < OBSTACLE_THRESHOLD_M:
                            action_taken = f"Obstacle Right ({right_dist:.2f}m)"
                            if right_x > (ROVER_HALF_WIDTH_M - SIDE_CLEARANCE_M):
                                action_taken += f" (X={right_x:.2f}m TOO CLOSE!) -> Reversing"
                                current_target_pwm_left = DRIVE_REVERSE_PWM
                                current_target_pwm_right = DRIVE_REVERSE_PWM
                            else:
                                action_taken += f" (X={right_x:.2f}m OK) -> Counter-Spinning Left"
                                current_target_pwm_left = DRIVE_REVERSE_PWM
                                current_target_pwm_right = DRIVE_FORWARD_PWM

                        print(action_taken)
                        # Update the main target variables
                        target_pwm_left = current_target_pwm_left
                        target_pwm_right = current_target_pwm_right

                    # else: Mismatch, don't update PWM targets, keep previous ones

                # --- Send Commands (Function now includes pause logic) ---
                # Send command every loop using the latest determined target PWMs
                if running:
                    send_diff_drive_override(master, target_pwm_left, target_pwm_right)
                else:
                    # stop_rover is handled in finally block
                    break

                # --- NEW: Display Frame with Overlays ---
                if frame is not None:
                    display_frame = frame.copy() # Work on a copy

                    # Draw ROIs
                    for name, r in ROIS:
                        # Get latest distance for this ROI, default to clear
                        dist_m, _ = obstacle_data.get(name, (999, 0))
                        color = OVERLAY_COLOR_CLEAR
                        if dist_m < OBSTACLE_THRESHOLD_M:
                            color = OVERLAY_COLOR_OBSTACLE # Red if obstacle detected

                        # Calculate pixel coordinates
                        top_left = (int(r.topLeft().x * PREVIEW_WIDTH), int(r.topLeft().y * PREVIEW_HEIGHT))
                        bottom_right = (int(r.bottomRight().x * PREVIEW_WIDTH), int(r.bottomRight().y * PREVIEW_HEIGHT))
                        # Draw rectangle and name
                        cv2.rectangle(display_frame, top_left, bottom_right, color, 2)
                        cv2.putText(display_frame, name, (top_left[0] + 5, top_left[1] + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                        # Optionally display distance inside ROI
                        cv2.putText(display_frame, f"{dist_m:.2f}m" if dist_m != 999 else "Inf",
                                    (top_left[0] + 5, top_left[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                    # Draw Status Text (Action)
                    cv2.putText(display_frame, action_taken, (10, PREVIEW_HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, OVERLAY_COLOR_TEXT, 1)

                    # Show the frame
                    cv2.imshow(PREVIEW_WINDOW_NAME, display_frame)

                # --- Check for Quit Key ---
                key = cv2.waitKey(1) # Check key press, crucial for imshow update
                if key == ord('q'):
                    print("'q' pressed, exiting.")
                    running = False
                    break # Exit loop immediately

                # --- Loop Delay ---
                # Adjust sleep time if needed, ensure it doesn't make loop too slow
                time.sleep(0.05) # Slightly reduced delay to make preview more responsive

    except Exception as e:
        print(f"An error occurred: {e}")
        traceback.print_exc()
    finally:
        print("Exiting script.")
        if master:
            if running: # Ensure stop is called if loop exited abnormally
                 stop_rover(master)
            master.close()
            print("MAVLink connection closed.")
        cv2.destroyAllWindows() # NEW: Close OpenCV window
        print("Script finished cleanly.")
