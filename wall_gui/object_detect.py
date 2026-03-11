from pathlib import Path
import time
import cv2
import depthai as dai
import numpy as np
import serial
import blobconverter

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 9600

labelMap = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
    "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

PERSON_LABEL = 15

ROW1_MASK = (1 << 0) | (1 << 1) | (1 << 2)
ROW2_MASK = ROW1_MASK | (1 << 3) | (1 << 4) | (1 << 5)
ROW3_MASK = 511

CONF_THRESH = 0.5
TRIGGER_HOLD_S = 2.0
DISPLAY_NAME = "Person Detection"

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
time.sleep(2)

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
nn = pipeline.create(dai.node.MobileNetDetectionNetwork)
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("nn")

# Camera setup
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setFps(30)

# Small preview for NN
camRgb.setPreviewSize(300, 300)

# Large video stream for display
camRgb.setIspScale(1, 1)           # full sensor scaling
camRgb.setVideoSize(1920, 1080)    # biggest practical display stream here

nn.setConfidenceThreshold(CONF_THRESH)
nn.setBlobPath(blobconverter.from_zoo(name="mobilenet-ssd", shaves=5))
nn.setNumInferenceThreads(2)
nn.input.setBlocking(False)

# NN still uses small preview
camRgb.preview.link(nn.input)

# Display uses large video stream
camRgb.video.link(xoutRgb.input)
nn.out.link(xoutNN.input)

last_sent_mask = None
current_mask = 0
hold_until = 0.0


def frame_norm(frame, bbox):
    norm_vals = np.full(len(bbox), frame.shape[0])
    norm_vals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * norm_vals).astype(int)


def person_count_to_mask(count):
    if count <= 0:
        return 0
    if count == 1:
        return ROW1_MASK
    if count == 2:
        return ROW2_MASK
    return ROW3_MASK


cv2.namedWindow(DISPLAY_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(DISPLAY_NAME, 1400, 900)

with dai.Device(pipeline) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    qNN = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

    frame = None
    detections = []

    start_time = time.monotonic()
    counter = 0

    while True:
        inRgb = qRgb.tryGet()
        inNN = qNN.tryGet()

        if inRgb is not None:
            frame = inRgb.getCvFrame()
            frame = cv2.flip(frame, 1)

        if inNN is not None:
            detections = inNN.detections
            counter += 1

        person_detections = []
        for det in detections:
            if det.label == PERSON_LABEL and det.confidence >= CONF_THRESH:
                person_detections.append(det)

        now = time.time()
        detected_count = min(len(person_detections), 3)
        target_mask = person_count_to_mask(detected_count)

        if now >= hold_until:
            if target_mask != current_mask:
                current_mask = target_mask
                hold_until = now + TRIGGER_HOLD_S

        if current_mask != last_sent_mask:
            ser.write(f"{current_mask}\n".encode("ascii"))
            last_sent_mask = current_mask

        if frame is not None:
            draw_frame = frame.copy()

            for det in person_detections:
                xmin, ymin, xmax, ymax = frame_norm(draw_frame, (det.xmin, det.ymin, det.xmax, det.ymax))

                flipped_xmin = draw_frame.shape[1] - xmax
                flipped_xmax = draw_frame.shape[1] - xmin

                cv2.rectangle(draw_frame, (flipped_xmin, ymin), (flipped_xmax, ymax), (255, 0, 0), 2)
                cv2.putText(
                    draw_frame,
                    "person",
                    (flipped_xmin + 8, ymin + 20),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.8,
                    (255, 0, 0),
                    1
                )

            fps = counter / max(time.monotonic() - start_time, 1e-6)

            cv2.putText(
                draw_frame,
                f"People: {detected_count}",
                (15, 35),
                cv2.FONT_HERSHEY_TRIPLEX,
                1.0,
                (255, 255, 255),
                1
            )

            cv2.putText(
                draw_frame,
                f"NN fps: {fps:.2f}",
                (15, draw_frame.shape[0] - 20),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.8,
                (255, 255, 255),
                1
            )

            cv2.imshow(DISPLAY_NAME, draw_frame)

        if cv2.waitKey(1) == ord('q'):
            break

ser.write(b"0\n")
ser.close()
cv2.destroyAllWindows()