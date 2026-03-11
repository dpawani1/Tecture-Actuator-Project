#!/usr/bin/env python3

import time
import argparse
from collections import deque

import cv2
import depthai as dai
import serial
import blobconverter

from palm_detection import PalmDetection


def draw_grid(frame, rows, cols):
    h, w = frame.shape[:2]
    for c in range(1, cols):
        x = int(w * c / cols)
        cv2.line(frame, (x, 0), (x, h), 255, 1)
    for r in range(1, rows):
        y = int(h * r / rows)
        cv2.line(frame, (0, y), (w, y), 255, 1)


def cell_from_xy(x, y, frame_w, frame_h, rows, cols):
    c = int(x / (frame_w / cols))
    r = int(y / (frame_h / rows))
    c = max(0, min(cols - 1, c))
    r = max(0, min(rows - 1, r))
    return r, c, r * cols + c


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


def mask_to_indices(mask, n):
    return [i for i in range(n) if (mask >> i) & 1]


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--grid_rows", type=int, default=3)
    parser.add_argument("--grid_cols", type=int, default=3)
    parser.add_argument("--stable_frames", type=int, default=2)
    parser.add_argument("--max_palms", type=int, default=6)

    parser.add_argument("--serial_port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=9600)

    parser.add_argument("--video_size", type=int, default=640)
    parser.add_argument("--fps", type=int, default=30)

    args = parser.parse_args()

    GRID_R = args.grid_rows
    GRID_C = args.grid_cols
    N_CELLS = GRID_R * GRID_C
    STABLE_N = max(1, args.stable_frames)

    WIN = "Palm Multi Grid"

    # ---------------- Pipeline ----------------
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(args.fps)
    cam.setIspScale(2, 3)  # 720p from 1080p
    cam.setVideoSize(args.video_size, args.video_size)
    cam.setPreviewSize(128, 128)
    cam.setInterleaved(False)

    xout_cam = pipeline.create(dai.node.XLinkOut)
    xout_cam.setStreamName("cam")
    cam.video.link(xout_cam.input)

    nn = pipeline.create(dai.node.NeuralNetwork)
    nn.setBlobPath(
        blobconverter.from_zoo(
            name="palm_detection_128x128",
            zoo_type="depthai",
            shaves=6
        )
    )
    nn.input.setBlocking(False)
    cam.preview.link(nn.input)

    xout_nn = pipeline.create(dai.node.XLinkOut)
    xout_nn.setStreamName("palm_nn")
    nn.out.link(xout_nn.input)

    # ---------------- Serial ----------------
    ser = None
    try:
        ser = serial.Serial(args.serial_port, args.baud, timeout=0.05)
        time.sleep(2.0)
        print(f"[SERIAL] Connected at {args.baud}")
    except Exception as e:
        print(f"[SERIAL] Not connected: {e}")

    # ---------------- Stability ----------------
    palmDetection = PalmDetection()
    recent_masks = deque(maxlen=STABLE_N)
    stable_mask = 0
    last_sent_mask = None

    # ---------------- Window ----------------
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1000, 1000)

    # ---------------- Run ----------------
    with dai.Device(pipeline) as device:
        vidQ = device.getOutputQueue("cam", 4, False)
        palmQ = device.getOutputQueue("palm_nn", 4, False)

        while True:
            frame = vidQ.get().getCvFrame()
            h, w = frame.shape[:2]

            # Mirrored display
            disp = cv2.flip(frame, 1)
            draw_grid(disp, GRID_R, GRID_C)

            mask = 0
            display_cells = []

            palm_in = palmQ.tryGet()
            if palm_in is not None:
                boxes = palmDetection.decode(frame, palm_in)

                boxes = sorted(
                    boxes,
                    key=lambda b: (b[2] - b[0]) * (b[3] - b[1]),
                    reverse=True
                )[:args.max_palms]

                for (x1, y1, x2, y2) in boxes:
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # Original camera-space cell for actuator mapping
                    r, c, idx = cell_from_xy(cx, cy, w, h, GRID_R, GRID_C)
                    mask |= (1 << idx)

                    # Mirrored display-space box and center
                    fx1 = w - x2
                    fx2 = w - x1
                    fcx = w - cx

                    cv2.rectangle(disp, (fx1, y1), (fx2, y2), 255, 2)
                    cv2.circle(disp, (fcx, cy), 4, 255, -1)

                    # Mirrored display-space cell for visual highlight
                    dr, dc, _ = cell_from_xy(fcx, cy, w, h, GRID_R, GRID_C)
                    display_cells.append((dr, dc))

                # Highlight mirrored display cells so preview matches what user sees
                for dr, dc in display_cells:
                    highlight_cell(disp, dr, dc, GRID_R, GRID_C)

            # Stability filter
            recent_masks.append(mask)
            if len(recent_masks) == STABLE_N and all(v == recent_masks[0] for v in recent_masks):
                stable_mask = mask

                if ser and stable_mask != last_sent_mask:
                    ser.write(f"{stable_mask}\n".encode())
                    last_sent_mask = stable_mask
                    print("Sent:", stable_mask, "Cells:", mask_to_indices(stable_mask, N_CELLS))

            cv2.putText(
                disp,
                f"Mask:{stable_mask}  Cells:{mask_to_indices(stable_mask, N_CELLS)}",
                (10, 25),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.6,
                255
            )

            cv2.imshow(WIN, disp)

            if cv2.waitKey(1) == ord('q'):
                break

    if ser:
        ser.close()


if __name__ == "__main__":
    main()