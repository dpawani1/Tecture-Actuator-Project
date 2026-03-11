#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
import time
import argparse
from collections import deque
import serial

# VOC class IDs for this MobileNet-SSD blob
LABELS = {
    8: "cat",
    12: "dog",
    15: "person"
}

# Default blob path (expects blob in ./models/)
nnPathDefault = str((Path(__file__).resolve().parent / "models" / "mobilenet-ssd_openvino_2021.4_5shave.blob").resolve())

parser = argparse.ArgumentParser()
parser.add_argument('nnPath', nargs='?', help="Path to mobilenet detection network blob", default=nnPathDefault)

# Full-frame tracking ON by default (rectangle display)
parser.add_argument('-ff', '--full_frame', action="store_true",
                    help="Perform tracking on full RGB frame (recommended)",
                    default=True)

# ---- Grid / serial knobs ----
parser.add_argument('--grid_rows', type=int, default=3)
parser.add_argument('--grid_cols', type=int, default=3)
parser.add_argument('--stable_frames', type=int, default=3, help="Consecutive frames needed before cell changes")
parser.add_argument('--z_trigger_mm', type=int, default=0, help="Only 'active' if Z <= this (mm). 0 disables (recommended).")
parser.add_argument('--choose', choices=['closest', 'largest'], default='closest',
                    help="If multiple targets: choose closest Z or largest bbox area")

parser.add_argument('--serial_port', default='/dev/ttyACM0', help='Arduino serial port e.g. /dev/ttyACM0 or /dev/ttyUSB0')
parser.add_argument('--baud', type=int, default=115200, help='Arduino baud rate (must match Serial.begin)')

# ---- Display knobs ----
parser.add_argument('--display_scale', type=float, default=1.5, help="Scale factor for on-screen display only")
parser.add_argument('--window_w', type=int, default=1100, help="Initial window width")
parser.add_argument('--window_h', type=int, default=700, help="Initial window height")
parser.add_argument('--headless', action='store_true', help='Run without GUI (no cv2.imshow)', default=False)

# ---- Camera / NN knobs ----
parser.add_argument('--video_w', type=int, default=1280, help="Displayed/tracked video width (rectangle)")
parser.add_argument('--video_h', type=int, default=720, help="Displayed/tracked video height (rectangle)")
parser.add_argument('--conf', type=float, default=0.35, help="NN confidence threshold (0.35 helps cat/dog)")

args = parser.parse_args()

fullFrameTracking = args.full_frame
GRID_R = args.grid_rows
GRID_C = args.grid_cols
STABLE_N = max(1, args.stable_frames)
Z_TRIGGER = args.z_trigger_mm
CHOOSE = args.choose
DISPLAY_SCALE = max(0.1, args.display_scale)

WIN = "tracker + grid + serial"


def draw_grid(frame, rows, cols, thickness=1):
    h, w = frame.shape[:2]
    for c in range(1, cols):
        x = int(w * c / cols)
        cv2.line(frame, (x, 0), (x, h), 255, thickness)
    for r in range(1, rows):
        y = int(h * r / rows)
        cv2.line(frame, (0, y), (w, y), 255, thickness)


def cell_from_xy(x, y, frame_w, frame_h, rows, cols):
    x = max(0, min(frame_w - 1, x))
    y = max(0, min(frame_h - 1, y))
    c = int(x / (frame_w / cols))
    r = int(y / (frame_h / rows))
    c = max(0, min(cols - 1, c))
    r = max(0, min(rows - 1, r))
    idx = r * cols + c
    return r, c, idx


def highlight_cell(frame, r, c, rows, cols):
    h, w = frame.shape[:2]
    x0 = int(w * c / cols)
    x1 = int(w * (c + 1) / cols)
    y0 = int(h * r / rows)
    y1 = int(h * (r + 1) / rows)

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x1, y1), 255, -1)
    cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)
    cv2.rectangle(frame, (x0, y0), (x1, y1), 255, 2)


# ---------------- Pipeline ----------------
pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
objectTracker = pipeline.create(dai.node.ObjectTracker)

xoutRgb = pipeline.create(dai.node.XLinkOut)
trackerOut = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("preview")
trackerOut.setStreamName("tracklets")

# ---- Camera setup (OAK-D Lite) ----
# Use 720p mode + 1280x720 video to avoid aspect cropping / "zoomed" FOV.
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# NN input MUST be 300x300 for this blob
camRgb.setPreviewSize(300, 300)
camRgb.setPreviewKeepAspectRatio(False)

# Full-FOV rectangular stream for display + full-frame tracking
camRgb.setVideoSize(args.video_w, args.video_h)

# ---- Mono / depth setup ----
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

# ---- NN setup ----
spatialDetectionNetwork.setBlobPath(args.nnPath)
spatialDetectionNetwork.setConfidenceThreshold(args.conf)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Track ONLY cat/dog/person
objectTracker.setDetectionLabelsToTrack([8, 12, 15])
objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

# ---- Linking ----
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

# NN runs on preview (300x300)
camRgb.preview.link(spatialDetectionNetwork.input)

# ObjectTracker output frame to host (what you display)
objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
objectTracker.out.link(trackerOut.input)

# Full-frame tracking uses camRgb.video as the tracker frame (RECTANGLE)
if fullFrameTracking:
    camRgb.video.link(objectTracker.inputTrackerFrame)
    objectTracker.inputTrackerFrame.setBlocking(False)
    objectTracker.inputTrackerFrame.setQueueSize(2)
else:
    # If disabled, tracker frame is 300x300 (not recommended)
    spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

# Detections + detection frame for tracker
spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
spatialDetectionNetwork.out.link(objectTracker.inputDetections)

# Depth to spatial NN
stereo.depth.link(spatialDetectionNetwork.inputDepth)

# ---- Stability filter state ----
recent_cells = deque(maxlen=STABLE_N)
stable_cell = None

with dai.Device(pipeline) as device:
    preview = device.getOutputQueue("preview", 4, False)
    tracklets = device.getOutputQueue("tracklets", 4, False)

    # ---- Serial to Arduino ----
    ser = None
    try:
        ser = serial.Serial(args.serial_port, args.baud, timeout=0.05)
        time.sleep(2.0)  # allow Arduino reset
        print(f"[SERIAL] Connected to {args.serial_port} @ {args.baud}")
    except Exception as e:
        print(f"[SERIAL] Could not open {args.serial_port}: {e}")

    last_sent_cell = None

    # ---- OpenCV window setup ----
    if not args.headless:
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, args.window_w, args.window_h)

    startTime = time.monotonic()
    counter = 0
    fps = 0

    while True:
        imgFrame = preview.get()
        trk = tracklets.get()

        counter += 1
        now = time.monotonic()
        if (now - startTime) > 1:
            fps = counter / (now - startTime)
            counter = 0
            startTime = now

        frame = imgFrame.getCvFrame()
        h, w = frame.shape[:2]

        # Debug overlay: if this stays 0, NN/links are wrong
        cv2.putText(frame, f"tracklets: {len(trk.tracklets)}", (10, 45),
                    cv2.FONT_HERSHEY_TRIPLEX, 0.65, 255)

        draw_grid(frame, GRID_R, GRID_C, thickness=1)

        # Choose one active target among cat/dog/person
        chosen = None
        chosen_score = None

        for t in trk.tracklets:
            label = LABELS.get(t.label, None)
            if label is None:
                continue

            roi = t.roi.denormalize(w, h)
            x1 = int(roi.topLeft().x)
            y1 = int(roi.topLeft().y)
            x2 = int(roi.bottomRight().x)
            y2 = int(roi.bottomRight().y)

            Xmm = int(t.spatialCoordinates.x)
            Ymm = int(t.spatialCoordinates.y)
            Zmm = int(t.spatialCoordinates.z)

            # Draw bbox + XYZ
            cv2.putText(frame, label, (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.65, 255)
            cv2.putText(frame, f"ID: {t.id}", (x1 + 10, y1 + 42), cv2.FONT_HERSHEY_TRIPLEX, 0.55, 255)
            cv2.putText(frame, t.status.name, (x1 + 10, y1 + 62), cv2.FONT_HERSHEY_TRIPLEX, 0.55, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), 255, 2)

            cv2.putText(frame, f"X: {Xmm} mm", (x1 + 10, y1 + 82), cv2.FONT_HERSHEY_TRIPLEX, 0.55, 255)
            cv2.putText(frame, f"Y: {Ymm} mm", (x1 + 10, y1 + 102), cv2.FONT_HERSHEY_TRIPLEX, 0.55, 255)
            cv2.putText(frame, f"Z: {Zmm} mm", (x1 + 10, y1 + 122), cv2.FONT_HERSHEY_TRIPLEX, 0.55, 255)

            # Optional Z trigger (0 disables)
            if Z_TRIGGER > 0 and (Zmm <= 0 or Zmm > Z_TRIGGER):
                continue

            area = max(0, x2 - x1) * max(0, y2 - y1)
            if CHOOSE == 'closest':
                score = Zmm if Zmm > 0 else 10**9
                better = (chosen is None) or (score < chosen_score)
            else:
                score = area
                better = (chosen is None) or (score > chosen_score)

            if better:
                chosen = (label, x1, y1, x2, y2, Xmm, Ymm, Zmm)
                chosen_score = score

        if chosen is not None:
            label, x1, y1, x2, y2, Xmm, Ymm, Zmm = chosen
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            r, c, idx = cell_from_xy(cx, cy, w, h, GRID_R, GRID_C)

            recent_cells.append(idx)
            if len(recent_cells) == STABLE_N and all(v == recent_cells[0] for v in recent_cells):
                stable_cell = idx

                # Send stable cell to Arduino ONLY when it changes
                if ser is not None and stable_cell != last_sent_cell:
                    msg = f"{stable_cell}\n"
                    try:
                        ser.write(msg.encode('utf-8'))
                        last_sent_cell = stable_cell
                        print(f"[SERIAL] Sent cell {stable_cell} ({label}) XYZ(mm)=({Xmm},{Ymm},{Zmm})")
                    except Exception as e:
                        print(f"[SERIAL] write failed: {e}")

            use_idx = stable_cell if stable_cell is not None else idx
            use_r = use_idx // GRID_C
            use_c = use_idx % GRID_C
            highlight_cell(frame, use_r, use_c, GRID_R, GRID_C)

            cv2.circle(frame, (cx, cy), 5, 255, -1)
            cv2.putText(frame, f"{label} | cell idx{idx} (r{r} c{c}) | stable: {stable_cell}",
                        (10, 22), cv2.FONT_HERSHEY_TRIPLEX, 0.65, 255)
        else:
            recent_cells.clear()
            stable_cell = None
            last_sent_cell = None
            cv2.putText(frame, "No active cat/dog/person",
                        (10, 22), cv2.FONT_HERSHEY_TRIPLEX, 0.65, 255)

        cv2.putText(frame, f"NN fps: {fps:.2f}", (2, h - 8),
                    cv2.FONT_HERSHEY_TRIPLEX, 0.55, 255)

        # Display zoom (does NOT change NN input)
        if not args.headless:
            if DISPLAY_SCALE != 1.0:
                display_frame = cv2.resize(frame, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                                           interpolation=cv2.INTER_LINEAR)
            else:
                display_frame = frame

            cv2.imshow(WIN, display_frame)
            if cv2.waitKey(1) == ord('q'):
                break
        else:
            time.sleep(0.001)

    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass
