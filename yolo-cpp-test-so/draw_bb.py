import os
import threading

os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import cv2
import olympe
import time
from pathlib import Path

# Default direct Wi-Fi IP address for the ANAFI Ai
DRONE_IP = "192.168.42.1"

global_bb = None  # Global variable to hold the latest bounding box data
global_bb_lock = threading.Lock()  # Lock for thread-safe access to global_bb

DEFAULT_MISSION_PATH = (
    Path(__file__).resolve().parent
    / ".airsdk/out/yolo_test-anafi2_classic/images/com.parrot.missions.samples.yolo.tar.gz"
)

TIMEOUT = 0.5  # Timeout in seconds for waiting for yolo_detection events
BBOX_SOURCE_WIDTH = 1280
BBOX_SOURCE_HEIGHT = 720

YOLO_CLASS_NAMES = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    4: "airplane",
    5: "bus",
    6: "train",
    7: "truck",
    8: "boat",
    9: "traffic light",
    10: "fire hydrant",
    11: "stop sign",
    12: "parking meter",
    13: "bench",
    14: "bird",
    15: "cat",
    16: "dog",
    17: "horse",
    18: "sheep",
    19: "cow",
    20: "elephant",
    21: "bear",
    22: "zebra",
    23: "giraffe",
    24: "backpack",
    25: "umbrella",
    26: "handbag",
    27: "tie",
    28: "suitcase",
    29: "frisbee",
    30: "skis",
    31: "snowboard",
    32: "sports ball",
    33: "kite",
    34: "baseball bat",
    35: "baseball glove",
    36: "skateboard",
    37: "surfboard",
    38: "tennis racket",
    39: "bottle",
    40: "wine glass",
    41: "cup",
    42: "fork",
    43: "knife",
    44: "spoon",
    45: "bowl",
    46: "banana",
    47: "apple",
    48: "sandwich",
    49: "orange",
    50: "broccoli",
    51: "carrot",
    52: "hot dog",
    53: "pizza",
    54: "donut",
    55: "cake",
    56: "chair",
    57: "couch",
    58: "potted plant",
    59: "bed",
    60: "dining table",
    61: "toilet",
    62: "tv",
    63: "laptop",
    64: "mouse",
    65: "remote",
    66: "keyboard",
    67: "cell phone",
    68: "microwave",
    69: "oven",
    70: "toaster",
    71: "sink",
    72: "refrigerator",
    73: "book",
    74: "clock",
    75: "vase",
    76: "scissors",
    77: "teddy bear",
    78: "hair drier",
    79: "toothbrush",
}

class SimpleOlympeStream:
    def __init__(self, drone):
        # Initialize the drone controller class
        self.drone = drone
        self._printed_frame_format = False

    def start(self):
        # 1. Connect to the drone
        print("Connecting to ANAFI Ai...")
        assert self.drone.connect(retry=3), "Connection failed. Check your Wi-Fi link."

        # 2. Command the drone to open its video pipeline
        # Olympe's Pdraw API uses 'raw_cb' to deliver fully decoded frames.
        print("Starting live video stream...")
        assert self.drone.streaming.play(
            raw_cb=self._on_frame_received,
        ), "Failed to start live video stream."

    def _on_frame_received(self, olympe_frame):
        """ This method runs asynchronously at ~30 FPS for every frame """

        WINDOW_NAME = "Parrot ANAFI Ai - Pure Olympe Stream"

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1280, 720)
        cv2_frame = self._frame_to_bgr(olympe_frame)

        if cv2_frame is not None:
            frame_height, frame_width = cv2_frame.shape[:2]
            print(f"Received frame size: {frame_width}x{frame_height}")
        
            # Draw the bounding box if available
            with global_bb_lock:
                if global_bb is not None and len(global_bb) > 0:
                    for bb in global_bb:
                        x, y, w, h = (
                            self._scale_x(bb.x, frame_width),
                            self._scale_y(bb.y, frame_height),
                            self._scale_x(bb.width, frame_width),
                            self._scale_y(bb.height, frame_height),
                        )
                        print(f"Drawing bounding box: x={x}, y={y}, w={w}, h={h}")
                        cv2.rectangle(
                            cv2_frame,
                            (x, y),
                            (x + w, y + h),
                            (0, 255, 0),  # Green color for the bounding box
                            2,  # Thickness of the rectangle
                        )

                        # Draw the label and confidence score if available
                        yolo_class = YOLO_CLASS_NAMES.get(bb.class_id, "Unknown")
                        label = f"{yolo_class}: {bb.confidence:.2f}"
                        cv2.putText(
                            cv2_frame,
                            label,
                            (x, y - 10),  # Position above the bounding box
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,  # Font scale
                            (0, 255, 0),  # Green color for the text
                            2,  # Thickness of the text
                        )
                


            # Render the frame to a local window on your monitor
            display_frame = cv2.resize(cv2_frame, (1280, 720))
            cv2.imshow(WINDOW_NAME, display_frame)
            cv2.imshow(WINDOW_NAME, cv2_frame)
            
            # A 1ms waitKey is mandatory to give OpenCV time to refresh the GUI window
            cv2.waitKey(1)

    def _frame_to_bgr(self, olympe_frame):
        frame_format = {}

        # print(f"Received frame: {olympe_frame}")
        try:
            # Olympe's raw as_ndarray() path expects the frame to be packed first.
            # Reading metadata does that through VideoFrame._get_video_frame().
            
            olympe_frame.info()
            frame_format = dict(olympe_frame.format())
        except Exception:
            pass

        # print(f"Frame format: {frame_format}")
        # print(f"frame after try: {olympe_frame}")

        yuv_frame = olympe_frame.as_ndarray()
        if yuv_frame is None:
            return None

        if not self._printed_frame_format:
            print(f"Raw frame format: {frame_format}")
            self._printed_frame_format = True

        format_name = " ".join(str(value).lower() for value in frame_format.values())
        if "nv12" in format_name:
            conversion = cv2.COLOR_YUV2BGR_NV12
        elif "yv12" in format_name:
            conversion = cv2.COLOR_YUV2BGR_YV12
        else:
            conversion = cv2.COLOR_YUV2BGR_I420

        return cv2.cvtColor(yuv_frame, conversion)

    def _scale_x(self, value, frame_width):
        return int(round(value * frame_width / BBOX_SOURCE_WIDTH))

    def _scale_y(self, value, frame_height):
        return int(round(value * frame_height / BBOX_SOURCE_HEIGHT))

    def stop(self):
        print("Shutting down stream and disconnecting...")
        self.drone.streaming.stop()
        self.drone.disconnect()
        cv2.destroyAllWindows()

class BoundingBox:
    def __init__(self, class_id, confidence, x, y, width, height):
        self.class_id = class_id
        self.confidence = confidence
        self.x = x
        self.y = y
        self.width = width
        self.height = height

def _event_payload(event):
    args = getattr(event, "args", None)
    if callable(args):
        args = args()

    if isinstance(args, dict):
        if "yolo_detection" in args:
            return args["yolo_detection"]
        if {"class_id", "confidence", "x", "y", "width", "height"}.issubset(
            args
        ):
            return args

    return None


def _field(detection, name):
    if isinstance(detection, dict):
        return detection[name]
    return getattr(detection, name)


def _format_detection(detection):
    return (
        f"class_id={_field(detection, 'class_id')} "
        f"confidence={_field(detection, 'confidence'):.3f} "
        f"box=(x={_field(detection, 'x')}, y={_field(detection, 'y')}, "
        f"w={_field(detection, 'width')}, h={_field(detection, 'height')})"
    )

def listen_yolo_events(drone):
    global global_bb

    mission_path = Path(DEFAULT_MISSION_PATH).expanduser().resolve()
    if not mission_path.exists():
        raise FileNotFoundError(
            f"Mission archive not found: {mission_path}\n"
            "Build the mission first with `airsdk build`, or pass --mission-path."
        )
    
    print(f"Using mission archive: {mission_path}")
    with drone.mission.from_path(str(mission_path)):
        from olympe.airsdk.messages.parrot.missions.samples.yolo.Event import (
            YoloDetections,
        )

        print("Waiting for yolo_detection events. Press Ctrl-C to stop.")
        try:
            while drone.connected:
                
                expectation = drone(
                    YoloDetections(_policy="wait")
                ).wait(_timeout=TIMEOUT)

                if not expectation.success():
                    
                    with global_bb_lock:
                        global_bb = []  # Clear the bounding boxes if no event is received
                    print("No yolo_detections event received before timeout.")
                    continue


                 # TODO - Add logic to extract bounding box data from the event and update global_bb
                
                detections = []
                for event in expectation.matched_events():
                    print(f"Received yolo_detections event: {event}")
                    # print(f"class id : {event.args['class_id']}")
                    # print(f"type of event: {type(event)}")
                    # class_id = event.args['class_id']
                    # confidence = event.args['confidence']
                    # x = event.args['x']
                    # y = event.args['y']
                    # width = event.args['width']
                    # height = event.args['height']
                    # # acquire lock to update global_bb safely
                    # with global_bb_lock:
                    #     global global_bb
                    #     if global_bb is None:
                    #         global_bb = BoundingBox(class_id, confidence, x, y, width, height)
                    #         print(f"Initialized global bounding box: {global_bb.__dict__}")
                    #     else:
                    #         global_bb.class_id = class_id
                    #         global_bb.confidence = confidence
                    #         global_bb.x = x
                    #         global_bb.y = y
                    #         global_bb.width = width
                    #         global_bb.height = height
                    #         print(f"Updated global bounding box: {global_bb.__dict__}")
                    #loop through detections
                    for detection in event.args["detections"]:
                        class_id = detection['class_id']
                        confidence = detection['confidence']
                        x = detection['x']
                        y = detection['y']
                        width = detection['width']
                        height = detection['height']

                        detections.append(BoundingBox(class_id, confidence, x, y, width, height))
                        print(f"Received detection: class_id={class_id}, confidence={confidence}, x={x}, y={y}, width={width}, height={height}")

                # acquire lock to update global_bb safely
                with global_bb_lock:
                    global_bb = detections 
                
               

        except KeyboardInterrupt:
            print("Stopping listener.")


def main():
    # initialize the drone
    drone = olympe.Drone(DRONE_IP)

    # initialize the stream
    streamer = SimpleOlympeStream(drone)
    streamer.start()

    # initialize thread for listening to yolo_detection events
    thread = threading.Thread(target=listen_yolo_events, args=(drone,), daemon=True)
    thread.start()

    try:
        # Keep the main execution thread alive while the stream thread runs
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        streamer.stop()
        print("Stream closed cleanly.")

if __name__ == "__main__":
    main()
    
