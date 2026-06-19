# Copyright (C) 2026 icarus

import asyncio
import logging
import signal
import threading
import ulog

import libpomp
import msghub
from road_runner.cv_road import messages_msghub

from .PixelLocationFinder import PixelLocationFinder

# This is limited to 15 characters
PROCESS_NAME = b"pos_estimate"
CV_ROAD_SERVICE_ADDR = "unix:/tmp/road-runner-cv-road-service"

DEFAULT_FRAME_WIDTH = 4608
DEFAULT_FRAME_HEIGHT = 3456
FOV_DEGREES = 75.5

CAMERA_TILT_DEGREES = -90 + 28.3
AGL_ALTITUDE_METERS = 120
HEADING_DEGREES = 90
LATITUDE_DEGREES = 44.68
LONGITUDE_DEGREES = -0.70


class PositionEstimationEventHandler(messages_msghub.EventHandler):
    def __init__(self, logger):
        super().__init__()
        self.logger = logger
        self.location_finder = None
        self.frame_width = None
        self.frame_height = None

    def _get_location_finder(self, frame_width, frame_height):
        if (
            self.location_finder is None
            or self.frame_width != frame_width
            or self.frame_height != frame_height
        ):
            self.frame_width = frame_width
            self.frame_height = frame_height
            self.location_finder = PixelLocationFinder(
                frame_height,
                frame_width,
                FOV_DEGREES,
            )
            self.logger.info(
                "Position estimator configured for frame size %dx%d",
                frame_width,
                frame_height,
            )

        return self.location_finder

    def position_estimation_trigger(self, msg):
        self.logger.info("Position estimation trigger received")
        try:
            self._process_position_estimation_trigger(msg)
        except Exception:
            self.logger.exception("Failed to process position estimation trigger")

    def _process_position_estimation_trigger(self, msg):
        frame_width = msg.frame_width or DEFAULT_FRAME_WIDTH
        frame_height = msg.frame_height or DEFAULT_FRAME_HEIGHT
        location_finder = self._get_location_finder(frame_width, frame_height)

        if len(msg.detections) == 0:
            self.logger.info(
                "Position estimation trigger at %d ms: no detections",
                msg.timestamp_ms,
            )
            return

        self.logger.info(
            "Position estimation trigger at %d ms: %d detections",
            msg.timestamp_ms,
            len(msg.detections),
        )

        for detection in msg.detections:
            x_center = detection.x + detection.width / 2.0
            y_center = detection.y + detection.height / 2.0

            forward, side = location_finder.getRelativePositionFromPicture(
                x_center,
                y_center,
                CAMERA_TILT_DEGREES,
                AGL_ALTITUDE_METERS,
            )

            self.logger.info("Position estimation: 1st pass complete")
            north, east = location_finder.getNePositionFromPicture(
                x_center,
                y_center,
                CAMERA_TILT_DEGREES,
                AGL_ALTITUDE_METERS,
                HEADING_DEGREES,
            )
            self.logger.info("Position estimation: 2nd pass complete")
            lat, lon = location_finder.getLatLonPositionFromPicture(
                x_center,
                y_center,
                CAMERA_TILT_DEGREES,
                AGL_ALTITUDE_METERS,
                HEADING_DEGREES,
                LATITUDE_DEGREES,
                LONGITUDE_DEGREES,
            )

            self.logger.info("Position estimation has run")

            self.logger.info(f"Position estimation result: forward={forward:.2f} m, side={side:.2f} m, north={north:.6f}, east={east:.6f}, lat={lat:.6f}, lon={lon:.6f}")   


class CvRoadEventClient:
    def __init__(self, logger):
        self.logger = logger
        self.loop = None
        self.msghub = None
        self.channel = None
        self.handler = None
        self.thread = None
        self.stop_requested = threading.Event()

    def start(self):
        self.loop = libpomp.pomp_loop_new()
        self.msghub = msghub.MessageHub(self.loop)
        self.channel = self.msghub.start_client_channel(CV_ROAD_SERVICE_ADDR)
        self.handler = PositionEstimationEventHandler(self.logger)
        self.msghub.attach_message_handler(self.handler)

        self.thread = threading.Thread(
            target=self._run_loop,
            name="cv-road-msghub",
            daemon=True,
        )
        self.thread.start()
        self.logger.info("Listening for cv_road events on %s", CV_ROAD_SERVICE_ADDR)

    def stop(self):
        self.stop_requested.set()

        if self.loop is not None:
            libpomp.pomp_loop_wakeup(self.loop)

        if self.thread is not None:
            self.thread.join(timeout=2.0)
            self.thread = None

        if self.msghub is not None and self.handler is not None:
            self.msghub.detach_message_handler(self.handler)
            self.handler = None

        if self.msghub is not None and self.channel is not None:
            self.msghub.stop_channel(self.channel)
            self.channel = None

        self.msghub = None

        if self.loop is not None:
            libpomp.pomp_loop_destroy(self.loop)
            self.loop = None

    def _run_loop(self):
        while not self.stop_requested.is_set():
            libpomp.pomp_loop_wait_and_process(self.loop, 100)


async def service_main():
    # Initialisation code
    #
    # The service is automatically started by the drone when the mission is
    # loaded.
    ulog.setup_logging(PROCESS_NAME)
    logger = logging.getLogger('main')
    logger.info("Hello from pos_estimate")
    run = True
    def sig_handler(*_):
        nonlocal run
        run = False

    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, sig_handler)

    cv_road_events = CvRoadEventClient(logger)
    cv_road_events.start()
    

    # Loop code
    #
    # The service is assumed to run an infinite loop, and termination
    # requests are handled via a SIGTERM signal.
    # If your service exits before this SIGTERM is sent, it will be
    # considered as a crash, and the system will relaunch the service.
    # If this happens too many times, the system will no longer start the
    # service.
    while run:
        await asyncio.sleep(1)

    # Cleanup code
    #
    # When stopped by a SIGTERM, a service can use a short amount of time
    # for cleanup (typically closing opened files and ensuring that the
    # written data is coherent).
    logger.info("Cleaning up from pos_estimate")
    cv_road_events.stop()
    return 0


def main():
    asyncio.run(service_main())
