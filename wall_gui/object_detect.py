import time
import cv2
import depthai as dai
import numpy as np
import serial
import blobconverter

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 9600

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

# Camera
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(30)

# Keep NN input small
camRgb.setPreviewSize(300, 300)

# Keep display stream moderate
camRgb.setIspScale(2, 3)
camRgb.setVideoSize(640, 640)

# NN
nn.setConfidenceThreshold(CONF_THRESH)
nn.setBlobPath(blobconverter.from_zoo(name="mobilenet-ssd", shaves=5))
nn.setNumInferenceThreads(2)
nn.input.setBlocking(False)

# Link streams
camRgb.preview.link(nn.input)
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
cv2.resizeWindow(DISPLAY_NAME, 1000, 1000)

with dai.Device(pipeline) as device:
    # Keep queues tiny so old frames do not pile up
    qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    qNN = device.getOutputQueue(name="nn", maxSize=1, blocking=False)

    frame = None
    detections = []

    start_time = time.monotonic()
    counter = 0

    while True:
        # Always take only latest frame / latest NN result
        inRgb = qRgb.get()
        frame = inRgb.getCvFrame()
        frame = cv2.flip(frame, 1)

        inNN = qNN.tryGet()
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

        # Draw directly on frame instead of frame.copy()
        for det in person_detections:
            xmin, ymin, xmax, ymax = frame_norm(frame, (det.xmin, det.ymin, det.xmax, det.ymax))

            flipped_xmin = frame.shape[1] - xmax
            flipped_xmax = frame.shape[1] - xmin

            cv2.rectangle(frame, (flipped_xmin, ymin), (flipped_xmax, ymax), (255, 0, 0), 2)
            cv2.putText(
                frame,
                "person",
                (flipped_xmin + 8, ymin + 20),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                (255, 0, 0),
                1
            )

        fps = counter / max(time.monotonic() - start_time, 1e-6)

        cv2.putText(
            frame,
            f"People: {detected_count}",
            (10, 25),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.7,
            (255, 255, 255),
            1
        )

        cv2.putText(
            frame,
            f"NN fps: {fps:.2f}",
            (10, frame.shape[0] - 10),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.5,
            (255, 255, 255),
            1
        )

        cv2.imshow(DISPLAY_NAME, frame)

        if cv2.waitKey(1) == ord('q'):
            break

ser.write(b"0\n")
ser.close()
cv2.destroyAllWindows()