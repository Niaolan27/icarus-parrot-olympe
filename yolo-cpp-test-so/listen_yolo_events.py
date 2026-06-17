#!/usr/bin/env python3

import argparse
import os
from pathlib import Path

os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import olympe


DEFAULT_DRONE_IP = "10.202.0.1"
DEFAULT_MISSION_PATH = (
    Path(__file__).resolve().parent
    / ".airsdk/out/yolo_test-anafi2_classic/images/com.parrot.missions.samples.yolo.tar.gz"
)


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


def main():
    parser = argparse.ArgumentParser(
        description="Print the best YOLO bounding-box event received through Olympe."
    )
    parser.add_argument(
        "--drone-ip",
        default=DEFAULT_DRONE_IP,
        help=f"Drone IP address. Default: {DEFAULT_DRONE_IP}",
    )
    parser.add_argument(
        "--mission-path",
        default=str(DEFAULT_MISSION_PATH),
        help="Path to the built mission archive.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for each event before printing a timeout message.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Exit after receiving the first yolo_detection event.",
    )
    args = parser.parse_args()

    mission_path = Path(args.mission_path).expanduser().resolve()
    if not mission_path.exists():
        raise FileNotFoundError(
            f"Mission archive not found: {mission_path}\n"
            "Build the mission first with `airsdk build`, or pass --mission-path."
        )

    drone = olympe.Drone(args.drone_ip)

    with drone.mission.from_path(str(mission_path)):
        from olympe.airsdk.messages.parrot.missions.samples.yolo.Event import (
            YoloDetection,
        )

        print(f"Connecting to {args.drone_ip}...")
        if not drone.connect():
            raise RuntimeError(f"Failed to connect to drone at {args.drone_ip}")

        print("Waiting for yolo_detection events. Press Ctrl-C to stop.")
        try:
            while drone.connected:
                expectation = drone(
                    YoloDetection(_policy="wait")
                ).wait(_timeout=args.timeout)

                if not expectation.success():
                    print("No yolo_detection event received before timeout.")
                    continue

                for event in expectation.matched_events():
                    payload = _event_payload(event)
                    if payload is None:
                        print(f"Received yolo_detection event: {event}")
                    else:
                        print(
                            "Received yolo_detection event: "
                            f"{_format_detection(payload)}"
                        )

                if args.once:
                    break
        except KeyboardInterrupt:
            print("Stopping listener.")
        finally:
            drone.disconnect()


if __name__ == "__main__":
    main()
