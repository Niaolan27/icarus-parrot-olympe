import os
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = ("rtsp_transport;udp|fflags;nobuffer|flags;low_delay|err_detect;explode")

import cv2
import threading
import time
import json
from urllib.request import urlopen




class RTSPStream:
    def __init__(self, rtsp_url):
        '''
        - Initializes the threading lock and starts a daemon thread to capture the RTSP stream asynchronously
        '''
        self.rtsp_url = rtsp_url
        self.frame    = None
        self.running  = True
        self.lock     = threading.Lock()
        self._open_stream()
        self.thread   = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()

    def _open_stream(self):
        '''
        - Configures and opens the OpenCV VideoCapture with settings optimized for low latency
        '''
        print(f"Opening RTSP stream from {self.rtsp_url}")
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def capture_loop(self):
        '''
        - Continuously reads frames from the stream in the background and stores the latest one
        - Automatically attempts to reconnect if the stream is lost
        '''
        print(f"Starting RTSP stream capture from {self.rtsp_url}")
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.5)
                print("Stream not open, attempting to reconnect...")
                if self.running:
                    self._open_stream()
                continue
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.frame = frame.copy()
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()

    def get_last_frame(self):
        '''
        - Retrieves the most recently captured frame in a thread-safe manner
        - Returns the frame as a numpy array, or None if no frame is currently available
        '''
        with self.lock:
            return self.frame if self.frame is not None else None

    def stop(self):
        '''
        - Signals the capture thread to terminate and waits for it to shut down
        '''
        self.running = False
        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=2.0)


def scanDrone():
    '''
    - Fetches the list of active compatible drones (ANAFI or ANAFI_USA) from a local network API
    - Returns a tuple containing the lists of discovered IP addresses and their corresponding names
    '''
    ListeAdresseIp, ListeNames = [], []
    try:
        data = json.loads(urlopen("http://10.0.0.100:8001/objects?type=dronewifi").read())
        for key, val in data.get("dronewifi", {}).items():
            if val.get("droneType") in ["ANAFI", "ANAFI_USA"]:
                ListeAdresseIp.append(val["ip"])
                ListeNames.append(val["name"])
    except Exception:
        pass
    return ListeAdresseIp, ListeNames

if __name__ == "__main__":
    # Example usage
    # ips, names = scanDrone()  # This will return lists of drone IPs and names on the local network
    # for ip, name in zip(ips, names):
    #     print(f"Found drone: {name} at IP: {ip}")
    
    drone_ip = "10.0.14.33"
    s = RTSPStream(f"rtsp://{drone_ip}/live")
    while True:
        frame = s.get_last_frame()
        if frame is not None:
            cv2.imshow("Drone Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    s.stop()
    cv2.destroyAllWindows()
