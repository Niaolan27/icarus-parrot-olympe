# Running YOLO with AirSDK and NCNN

This mission runs YOLO directly inside an AirSDK C++ service. The model is
exported to NCNN format, bundled as mission assets, loaded at service startup,
and executed against frames received from the drone video pipeline.

The important idea is that inference happens in the mission service. Frames are
not streamed to a laptop for detection.

The current `mission.yaml` enables the `Anafi Ai Simulator` target by default.
The physical Anafi Ai and Anafi UKR target entries are present but commented
out; enable the right target only when you are ready to run on that hardware.
The service configuration keeps the legacy road-following control fields
motion-neutral, so this example logs detections and does not intentionally
command drone movement.

## What Is Included

- `deps/ncnn/` contains the NCNN runtime source. AirSDK builds it as an
  `alchemy` dependency.
- `deps/yolo_detector/` is a small C++ wrapper around NCNN. It loads the YOLO
  model, preprocesses frames, runs inference, decodes YOLO-style outputs, and
  applies non-maximum suppression.
- `assets/models/` is where exported NCNN model files belong before packaging.
  The default config expects `model.ncnn.param` and `model.ncnn.bin`.
- `assets/etc/services/cv_road.cfg` configures the model paths, input size,
  blob names, thresholds, position-estimation trigger period, and neutral
  legacy road-following fields.
- `services/cv_road/` is the AirSDK service that receives camera frames through
  video IPC, calls the YOLO detector, and publishes detection events.
- `services/position_estimation/` is the AirSDK Python service that converts
  detection pixel locations into relative ground positions and estimated
  latitude/longitude values.

## Build Wiring

The AirSDK build is controlled by `mission.yaml`.

NCNN is declared as a custom dependency:

```yaml
deps:
  ncnn:
    type: alchemy
```

The YOLO detector is declared as a C++ dependency that links against NCNN and
OpenCV:

```yaml
  yolo_detector:
    lang: c++
    headers: include
    depends:
      - libulog
      - ncnn
      - opencv4
```

The `cv_road` service then depends on `yolo_detector`, so the detector code is
compiled into the service that runs on the drone:

```yaml
services:
  cv_road:
    lang: c++
    depends:
      - libconfigreader
      - libfutils
      - libmsghub
      - libputils
      - libtelemetry
      - libvideo-ipc
      - libvideo-ipc-client-config
      - msghub::cv_road
      - opencv4
      - protobuf
      - yolo_detector
```

The `position_estimation` service is a Python service. It subscribes to
`cv_road` events and publishes its own results through a second message-hub
channel:

```yaml
services:
  position_estimation:
    lang: python
    depends:
      - libmsghub
      - msghub::mission_ui
      - msghub::cv_road
      - msghub::position_estimation
```

## Exporting A YOLO Model To NCNN

Export the trained YOLO model into NCNN's two-file format:

- `model.ncnn.param`
- `model.ncnn.bin`

For Ultralytics YOLO models, the typical export command is:

```bash
yolo export model=path/to/best.pt format=ncnn imgsz=320
```

Then copy the exported files into:

```text
assets/models/model.ncnn.param
assets/models/model.ncnn.bin
```

If the exported files use different names, either rename them to match the
current config or update `assets/etc/services/cv_road.cfg`.

Model binaries can be large. If you do not want to keep them in Git, keep a
placeholder such as `assets/models/.gitkeep`, ignore the generated model files,
and copy the real `.param` and `.bin` files into `assets/models/` before
building the mission.

## Model Configuration

The service reads YOLO settings from `assets/etc/services/cv_road.cfg`:

```cfg
yoloParamPath = "models/model.ncnn.param";
yoloBinPath = "models/model.ncnn.bin";
yoloInputBlob = "";
yoloOutputBlob = "";
yoloInputWidth = 320;
yoloInputHeight = 192;
yoloConfidenceThreshold = 0.50;
yoloNmsThreshold = 0.45;
STREAM_YOLO_DETECTIONS = true;
positionEstimationTriggerPeriodSeconds = 0.50;
```

Paths are resolved against the installed mission root. During packaging, files
from `assets/` are copied into that root, so the config path
`models/model.ncnn.bin` resolves to the file that was placed locally at
`assets/models/model.ncnn.bin`.

Leaving `yoloInputBlob` and `yoloOutputBlob` empty tells the detector to use the
first input and first output exposed by the NCNN model. If an exported model has
multiple inputs or outputs, set these fields to the exact blob names from the
NCNN `.param` file.

`yoloInputWidth` and `yoloInputHeight` must match the size used during export.
The detector letterboxes each camera frame into that size before inference.

Set `STREAM_YOLO_DETECTIONS` to `true` to publish `yolo_detections` events to
Olympe, or `false` to keep detections local to service logs.

`positionEstimationTriggerPeriodSeconds` controls how often `cv_road` sends a
batch of detections to the position-estimation service. Fractional values are
supported, so `0.50` means one trigger every half second. Set it to `0` or a
negative value to disable position-estimation triggers.

The other fields in the `road_following` section are legacy fields still read by
the service. In this example they are set to neutral values.

## Position Estimation Algorithm

Position estimation starts in `cv_road`. When the trigger period elapses, the
service packages the current frame timestamp, frame size, and YOLO detections
into a `position_estimation_trigger` event. Each detection contains the class id,
confidence, and bounding box in image pixels.

The Python `position_estimation` service listens for those trigger events. For
each detection, it uses the bounding-box center:

```text
x_center = x + width / 2
y_center = y + height / 2
```

`PixelLocationFinder` then treats that center pixel as a camera ray. The helper
builds a simple pinhole-camera model from the frame width, frame height, and
horizontal field of view. Pixel coordinates are normalized around the image
center, converted into a ray in camera coordinates, pitched by the camera tilt,
and intersected with the ground plane at the configured above-ground altitude.

The first result is a local position relative to the drone's nadir point:

- `forward_m`: distance in front of the drone/camera projection.
- `side_m`: lateral distance from that projection.

The same local vector is rotated by the configured heading to produce
north/east offsets:

- `north_m`
- `east_m`

Finally, the north/east offset is added to the configured reference GPS point
using an Earth-radius approximation. The published result contains both the
relative position and the estimated geographic position:

- `timestamp_ms`
- `class_id`
- `confidence`
- `x_center`, `y_center`
- `forward_m`, `side_m`
- `north_m`, `east_m`
- `latitude`, `longitude`

The current implementation uses constants in
`services/position_estimation/main.py`:

```python
FOV_DEGREES = 68.0
CAMERA_TILT_DEGREES = -80.0
AGL_ALTITUDE_METERS = 120
HEADING_DEGREES = 90
LATITUDE_DEGREES = 44.68
LONGITUDE_DEGREES = -0.70
```

These values are placeholders for the camera model and drone state. For accurate
positions, replace them with telemetry-backed values for the real frame: camera
field of view, gimbal pitch, AGL altitude, heading, latitude, and longitude.
Errors in tilt or altitude directly move the ground intersection, so the output
should be treated as an estimate until those inputs are tied to live telemetry.

## Runtime Flow

1. AirSDK starts the `cv_road` C++ service in the selected target environment.
2. AirSDK starts the `position_estimation` Python service.
3. `Processing` loads `cv_road.cfg`.
4. The configured NCNN `.param` and `.bin` files are loaded by
   `yolo_detector::Detector`.
5. The service receives video frames from video IPC.
6. The frame is converted from the drone YUV format into an OpenCV image.
7. The detector resizes and letterboxes the image, normalizes pixels, and sends
   it through NCNN.
8. YOLO detections are decoded into class id, confidence, and bounding box.
9. If streaming is enabled, detections are published as `yolo_detections`.
10. At the configured trigger period, `cv_road` sends detections and frame
   metadata to `position_estimation`.
11. `position_estimation` converts detection centers into relative, north/east,
    and latitude/longitude estimates, then publishes
    `position_estimation_results`.
12. The flight supervisor relays YOLO detections and position-estimation results
    to the Olympe-facing mission messages.

The current detector runs NCNN on CPU:

```cpp
net.opt.use_vulkan_compute = false;
net.opt.num_threads = 2;
```

That keeps the runtime simple for on-drone execution and avoids depending on a
GPU/Vulkan path being available in the AirSDK environment.

## Building And Installing

From this mission directory:

```bash
airsdk build
airsdk install --default
```

For simulator use, make sure the simulator target selected in `mission.yaml`
matches the AirSDK environment you are installing to. After installing on a
physical drone, give the drone time to reboot and start the mission services
before checking logs.

## Verifying On The Drone

Use the drone shell or your usual AirSDK logging workflow and look for `ulog`
messages from these tags:

- `yolo_detector`
- `processing`
- `service_main`

Useful messages include:

- `loaded NCNN YOLO model param=... bin=...`
- `NCNN output shape: dims=... w=... h=... c=...`
- `YOLO candidates before NMS: ...`
- `YOLO detections: ...`
- `YOLO class=... confidence=... box=[x=...,y=...,w=...,h=...]`

If the model fails to load, check that the files exist in `assets/models/`, that
the paths in `cv_road.cfg` match, and that the exported NCNN files are compatible
with the NCNN source version in `deps/ncnn/`.

If the service starts but logs `YOLO detector is not ready`, the model load
failed earlier in startup. Look above that warning for the exact NCNN load
error and resolved file path.

## Visualizing YOLO detections on host laptop

You can set the `STREAM_YOLO_DETECTIONS` flag in `cv_road.cfg` to true if you want
to stream the YOLO detections from the drone back to a client computer. 

`draw_bb.py` is a Python script that captures the video stream from the drone. It also
listens to YOLO detectioncevents being published, and overlays the bounding boxes
over the video frames. 

The client computer has to be connected to the drone Wifi for this to work.

One thing to note, the YOLO inference is very slow on the drone. Hence, the 
bounding boxes arrive later than the corresponding video frame. If the drone is 
moving, it will look like the bounding boxes are in the wrong position.

## Swapping Models

To run a different YOLO model:

1. Export the new model to NCNN using the same image size you want to run on the
   drone.
2. Replace `assets/models/model.ncnn.param` and
   `assets/models/model.ncnn.bin`, or update `cv_road.cfg` with new file names.
3. Adjust `yoloInputWidth`, `yoloInputHeight`, `yoloConfidenceThreshold`, and
   `yoloNmsThreshold` as needed.
4. If the model's input or output blob cannot be inferred, set
   `yoloInputBlob` and `yoloOutputBlob`.
5. Rebuild and reinstall the mission.

## Notes

- Keep the model small enough for the drone CPU. Start with a nano/small YOLO
  model and increase only if frame rate and thermal behavior are acceptable.
- The current decoder expects standard YOLO detection output shaped like
  `[x, y, w, h, class scores...]`, including common YOLOv8/YOLO11 NCNN exports.
- Segmentation, pose, oriented-box, or custom heads need additional decode logic
  in `deps/yolo_detector/src/detector.cpp`.
- The service currently logs detections. To make YOLO drive behavior, route the
  detections into the mission's guidance or message-hub path.
