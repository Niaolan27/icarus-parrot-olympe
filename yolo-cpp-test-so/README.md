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
  blob names, thresholds, and neutral legacy road-following fields.
- `services/cv_road/` is the AirSDK service that receives camera frames through
  video IPC and calls the YOLO detector.

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

## Exporting A YOLO Model To NCNN

Export the trained YOLO model into NCNN's two-file format:

- `model.ncnn.param`
- `model.ncnn.bin`

For Ultralytics YOLO models, the typical export command is:

```bash
yolo export model=path/to/best.pt format=ncnn imgsz=640
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
yoloInputWidth = 640;
yoloInputHeight = 640;
yoloConfidenceThreshold = 0.1;
yoloNmsThreshold = 0.45;
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

The other fields in the `road_following` section are legacy fields still read by
the service. In this example they are set to neutral values.

## Runtime Flow

1. AirSDK starts the `cv_road` C++ service in the selected target environment.
2. `Processing` loads `cv_road.cfg`.
3. The configured NCNN `.param` and `.bin` files are loaded by
   `yolo_detector::Detector`.
4. The service receives video frames from video IPC.
5. The frame is converted from the drone YUV format into an OpenCV image.
6. The detector resizes and letterboxes the image, normalizes pixels, and sends
   it through NCNN.
7. YOLO detections are decoded into class id, confidence, and bounding box.
8. Detections are logged with `ulog`; they are not currently used to drive a
   guidance behavior.

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
