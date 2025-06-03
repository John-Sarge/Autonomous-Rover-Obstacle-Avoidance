#!/usr/bin/env python3

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import json # For parsing the JSON configuration file
import roslibpy # For ROS communication via rosbridge
import math     # For math.radians()

# --- Configuration ---
# Path to the neural network blob file
NN_BLOB_PATH = str((Path(__file__).parent / Path('best.blob')).resolve().absolute())
if not Path(NN_BLOB_PATH).exists():
    sys.exit(f"Required NN blob file not found: {NN_BLOB_PATH}")

# Path to the JSON configuration file for the NN
NN_CONFIG_PATH = str((Path(__file__).parent / Path('best.json')).resolve().absolute())
if not Path(NN_CONFIG_PATH).exists():
    sys.exit(f"Required NN configuration file not found: {NN_CONFIG_PATH}")

# Parse JSON configuration file
try:
    with open(NN_CONFIG_PATH, 'r') as f:
        nn_config = json.load(f)
except Exception as e:
    sys.exit(f"Error parsing JSON configuration file: {e}")

# Extract NN specific parameters from JSON
nn_input_size_str_list = nn_config.get("nn_config", {}).get("input_size", "640x640").split('x')
try:
    if not nn_input_size_str_list or not nn_input_size_str_list[0]: 
        raise ValueError("input_size string is empty or the first dimension is missing/empty after split.")
    NN_INPUT_WIDTH = int(nn_input_size_str_list[0])
    if len(nn_input_size_str_list) >= 2 and nn_input_size_str_list[1]: 
        NN_INPUT_HEIGHT = int(nn_input_size_str_list[1])
    else: 
        NN_INPUT_HEIGHT = NN_INPUT_WIDTH
except (ValueError, IndexError) as e:
    print(f"Warning: Invalid input_size '{nn_config.get('nn_config', {}).get('input_size', '640x640')}' in JSON. Error: {e}. Defaulting to 640x640.")
    NN_INPUT_WIDTH = 640
    NN_INPUT_HEIGHT = 640

labels = nn_config.get("mappings", {}).get("labels", ["redball"])
if not isinstance(labels, list) or len(labels) == 0:
    print("Warning: Labels not found or empty in JSON config. Using default ['redball'].")
    labels = ["redball"]
labelMap = labels
CONFIDENCE_THRESHOLD = 0.5

# Define depth thresholds (in mm) and store them
DEPTH_LOWER_THRESHOLD_MM = 10  # Min range 10cm
DEPTH_UPPER_THRESHOLD_MM = 10000 # Max range 10m

# --- ROS2 Communication Setup ---
ROS_BRIDGE_HOST = 'localhost'
ROS_BRIDGE_PORT = 9090
FRAME_ID = "oak_rgb_camera_optical_frame"
OAK_D_MONO_HFOV_DEG = 72.0

ros_client = None 
range_publisher = None
RANGE_TOPIC_NAME = '/redball/range'
RANGE_MSG_TYPE = 'sensor_msgs/Range'

ros_client_singleton = None
ros_reactor_started = False

def initialize_ros_client():
    global ros_client, range_publisher, ros_client_singleton, ros_reactor_started

    if ros_client_singleton and ros_client_singleton.is_connected:
        ros_client = ros_client_singleton
        if range_publisher is None or range_publisher.client != ros_client:
            range_publisher = roslibpy.Topic(ros_client, RANGE_TOPIC_NAME, RANGE_MSG_TYPE)
        if not range_publisher.is_advertised:
            try:
                range_publisher.advertise()
            except Exception as e:
                print(f"Error re-advertising topic on existing connection: {e}")
                if not ros_client_singleton.is_connected:
                    ros_client = None
                    return False
        return True

    if ros_client_singleton is None:
        print("Creating ROS client instance for the first time.")
        ros_client_singleton = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    
    if not ros_reactor_started:
        print(f"Attempting to start ROS reactor and connect to {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
        try:
            ros_client_singleton.run() 
            ros_reactor_started = True
            time.sleep(2)
        except Exception as e:
            print(f"Fatal error during initial ros_client.run(): {e}. ROS functionality will be unavailable.")
            ros_client_singleton = None 
            ros_reactor_started = False
            ros_client = None
            return False
    elif not ros_client_singleton.is_connected:
        print(f"ROS reactor running, attempting ros_client.connect() to {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
        try:
            ros_client_singleton.connect()
            time.sleep(1)
        except roslibpy.core.RosTimeoutError:
            print(f"Timeout during ros_client.connect(). Will retry later.")
            ros_client = None
            return False
        except Exception as e:
            print(f"Error during ros_client.connect(): {e}")
            ros_client = None
            return False

    if ros_client_singleton and ros_client_singleton.is_connected:
        ros_client = ros_client_singleton
        print("Successfully connected/re-connected to ROS Bridge Server.")
        if range_publisher is None or range_publisher.client != ros_client:
            range_publisher = roslibpy.Topic(ros_client, RANGE_TOPIC_NAME, RANGE_MSG_TYPE)
        
        if not range_publisher.is_advertised:
            try:
                range_publisher.advertise()
                print(f"ROS topic: {RANGE_TOPIC_NAME} of type {RANGE_MSG_TYPE} is ready.")
            except Exception as e:
                print(f"Error advertising topic after connection: {e}")
                if not ros_client_singleton.is_connected:
                    ros_client = None
                    return False
        return True
    else:
        if ros_reactor_started:
            print(f"Failed to connect to ROS Bridge Server (reactor running). Will retry later.")
        else:
            print(f"Failed to connect to ROS Bridge Server (reactor not started).")
        ros_client = None
        return False

# --- Helper Functions ---
def frameNorm(frame_shape, bbox):
    normVals = np.array([frame_shape[1], frame_shape[0], frame_shape[1], frame_shape[0]])
    return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

# --- Main Pipeline Setup and Execution ---
if __name__ == '__main__':
    initialize_ros_client()

    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setPreviewSize(NN_INPUT_WIDTH, NN_INPUT_HEIGHT)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(15)

    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)

    spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    spatialDetectionNetwork.setBlobPath(NN_BLOB_PATH)
    spatialDetectionNetwork.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    # Use the stored constants to set depth thresholds
    spatialDetectionNetwork.setDepthLowerThreshold(DEPTH_LOWER_THRESHOLD_MM)
    spatialDetectionNetwork.setDepthUpperThreshold(DEPTH_UPPER_THRESHOLD_MM)
    spatialDetectionNetwork.setNumClasses(len(labelMap))
    spatialDetectionNetwork.setCoordinateSize(4)
    spatialDetectionNetwork.setIouThreshold(nn_config.get("nn_config", {}).get("iou_threshold", 0.5))

    camRgb.preview.link(spatialDetectionNetwork.input)
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    stereo.depth.link(spatialDetectionNetwork.inputDepth)

    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    camRgb.preview.link(xoutRgb.input) 

    xoutSpatialData = pipeline.create(dai.node.XLinkOut)
    xoutSpatialData.setStreamName("spatialData")
    spatialDetectionNetwork.out.link(xoutSpatialData.input)

    print(f"Pipeline created. Connecting to OAK-D device...")
    try:
        with dai.Device(pipeline) as device:
            print(f"Connected to OAK-D. Starting pipeline...")
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            spatialDataQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)

            startTime = time.monotonic()
            counter = 0
            fps = 0
            
            last_ros_connection_attempt_time = time.monotonic()
            ROS_RECONNECTION_INTERVAL = 5 # seconds

            print("Entering main loop...")
            while True:
                current_loop_time = time.monotonic()
                
                if not (ros_client and ros_client.is_connected):
                    if (current_loop_time - last_ros_connection_attempt_time) > ROS_RECONNECTION_INTERVAL:
                        print("ROS client not connected. Attempting to (re)connect...")
                        initialize_ros_client()
                        last_ros_connection_attempt_time = current_loop_time

                inPreview = previewQueue.tryGet()
                frame = None
                if inPreview is not None:
                    frame = inPreview.getCvFrame()

                detections = None 
                inSpatialData = spatialDataQueue.tryGet()
                if inSpatialData is not None:
                    detections = inSpatialData.detections

                if frame is None and (detections is None or not detections): 
                    if cv2.waitKey(1) == ord('q'): 
                        break
                    time.sleep(0.01) 
                    continue
                
                counter += 1
                if (current_loop_time - startTime) >= 1:
                    fps = counter / (current_loop_time - startTime)
                    counter = 0
                    startTime = current_loop_time

                if detections is not None:
                    for detection in detections: 
                        if ros_client and ros_client.is_connected and range_publisher:
                            if not range_publisher.is_advertised: 
                                try:
                                    range_publisher.advertise()
                                except Exception as e:
                                    print(f"Error advertising topic during runtime publish check: {e}")
                                    if not ros_client.is_connected: ros_client = None
                            
                            if ros_client and range_publisher.is_advertised:
                                current_time_ros = time.time()
                                ros_msg_timestamp_sec = int(current_time_ros)
                                ros_msg_timestamp_nanosec = int((current_time_ros % 1) * 1e9)
                                range_in_meters = detection.spatialCoordinates.z / 1000.0
                                
                                # Use the stored constants for min/max range
                                min_range_m = DEPTH_LOWER_THRESHOLD_MM / 1000.0
                                max_range_m = DEPTH_UPPER_THRESHOLD_MM / 1000.0
                                
                                range_msg_dict = {
                                    'header': {
                                        'stamp': {'sec': ros_msg_timestamp_sec, 'nanosec': ros_msg_timestamp_nanosec},
                                        'frame_id': FRAME_ID
                                    },
                                    'radiation_type': 1,
                                    'field_of_view': math.radians(OAK_D_MONO_HFOV_DEG), 
                                    'min_range': min_range_m,
                                    'max_range': max_range_m,
                                    'range': range_in_meters if min_range_m <= range_in_meters <= max_range_m and detection.spatialCoordinates.z != 0 else float('inf')
                                }
                                ros_range_message = roslibpy.Message(range_msg_dict)
                                try:
                                    range_publisher.publish(ros_range_message)
                                except Exception as e:
                                    print(f"Error publishing ROS message: {e}")
                                    if not ros_client.is_connected:
                                        print("ROS client disconnected during publish. Will attempt to reconnect.")
                                        ros_client = None
                        
                        if frame is not None: 
                            bbox_pixel = frameNorm(frame.shape[:2], (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                            xmin, ymin, xmax, ymax = bbox_pixel
                            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                            centerX = int((xmin + xmax) / 2)
                            centerY = int((ymin + ymax) / 2)
                            cv2.circle(frame, (centerX, centerY), 5, (0, 0, 255), -1)
                            try:
                                labelText = labelMap[detection.label]
                            except IndexError:
                                labelText = f"Label {detection.label}"
                            cv2.putText(frame, f"{labelText}: {detection.confidence:.2f}",
                                        (xmin, ymin - 10 if ymin - 10 > 10 else ymin + 20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            spatialCoords = detection.spatialCoordinates
                            x_m = spatialCoords.x / 1000.0
                            y_m = spatialCoords.y / 1000.0
                            z_m = spatialCoords.z / 1000.0 
                            text_y_offset = ymax + 20
                            cv2.putText(frame, f"X: {x_m:.2f}m", (xmin, text_y_offset), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255), 1)
                            cv2.putText(frame, f"Y: {y_m:.2f}m", (xmin, text_y_offset + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255), 1)
                            cv2.putText(frame, f"Z: {z_m:.2f}m ({spatialCoords.z:.0f}mm)", (xmin, text_y_offset + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,0,0), 1)

                if frame is not None:
                    cv2.putText(frame, "FPS: {:.2f}".format(fps), (20, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,255,0), 2)
                    cv2.imshow("Red Ball Detection - OAK-D with ROS Publishing", frame)

                if cv2.waitKey(1) == ord('q'):
                    break
    
    except Exception as e:
        print(f"An error occurred in the main OAK-D execution block: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Exiting application...")
        if ros_client_singleton: 
            if ros_client_singleton.is_connected:
                print("Terminating ROS client connection...")
            else:
                print("ROS client was not connected, but attempting to terminate its resources.")
            try:
                ros_client_singleton.terminate() 
                if hasattr(ros_client_singleton, 'thread') and ros_client_singleton.thread is not None:
                    ros_client_singleton.thread.join(timeout=2.0)
                print("ROS client resources terminated.")
            except Exception as e:
                print(f"Error terminating ROS client: {e}")
        else:
            print("ROS client was not initialized. No termination needed.")
        
        cv2.destroyAllWindows()
