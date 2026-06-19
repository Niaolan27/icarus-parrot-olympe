import os

os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import cv2
import olympe
import time

# Default direct Wi-Fi IP address for the ANAFI Ai
DRONE_IP = "192.168.42.1"

class SimpleOlympeStream:
    def __init__(self):
        # Initialize the drone controller class
        self.drone = olympe.Drone(DRONE_IP)
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
        cv2_frame = self._frame_to_bgr(olympe_frame)

        if cv2_frame is not None:
            # Render the frame to a local window on your monitor
            cv2.imshow("Parrot ANAFI Ai - Pure Olympe Stream", cv2_frame)
            
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

    def stop(self):
        print("Shutting down stream and disconnecting...")
        self.drone.streaming.stop()
        self.drone.disconnect()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    streamer = SimpleOlympeStream()
    streamer.start()

    try:
        # Keep the main execution thread alive while the stream thread runs
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        streamer.stop()
        print("Stream closed cleanly.")
