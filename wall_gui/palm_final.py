#!/usr/bin/env python3

import time
import argparse

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
    parser.add_argument("--max_palms", type=int, default=6)

    parser.add_argument("--serial_port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=9600)

    parser.add_argument("--video_width", type=int, default=960)
    parser.add_argument("--video_height", type=int, default=640)
    parser.add_argument("--fps", type=int, default=30)

    parser.add_argument("--hold_detect_s", type=float, default=2.0)
    parser.add_argument("--actuate_hold_s", type=float, default=4.0)

    parser.add_argument("--trail_time_s", type=float, default=2.0)
    parser.add_argument("--trail_max_points", type=int, default=120)

    args = parser.parse_args()

    GRID_R = args.grid_rows
    GRID_C = args.grid_cols
    N_CELLS = GRID_R * GRID_C

    WIN = "Palm Multi Grid"

    # ---------------- Pipeline ----------------
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(args.fps)
    cam.setIspScale(2, 3)
    cam.setVideoSize(args.video_width, args.video_height)
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

    palmDetection = PalmDetection()

    # For each cell:
    # detect_start[i] = time when hand started staying in that cell
    # active_until[i] = time until actuator stays on
    detect_start = [None] * N_CELLS
    active_until = [0.0] * N_CELLS

    last_sent_mask = None
    trail_points = []

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1200, 800)

    with dai.Device(pipeline) as device:
        vidQ = device.getOutputQueue("cam", 4, False)
        palmQ = device.getOutputQueue("palm_nn", 4, False)

        while True:
            now = time.time()

            frame = vidQ.get().getCvFrame()
            h, w = frame.shape[:2]

            disp = cv2.flip(frame, 1)
            draw_grid(disp, GRID_R, GRID_C)

            detected_cells = set()
            display_detect_cells = set()
            current_centers = []

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

                    # actuator-space mapping
                    r, c, _ = cell_from_xy(cx, cy, w, h, GRID_R, GRID_C)
                    mapped_r = GRID_R - 1 - r
                    idx = mapped_r * GRID_C + c
                    detected_cells.add(idx)

                    # mirrored display drawing
                    fx1 = w - x2
                    fx2 = w - x1
                    fcx = w - cx

                    cv2.rectangle(disp, (fx1, y1), (fx2, y2), 255, 2)
                    cv2.circle(disp, (fcx, cy), 4, 255, -1)

                    # display-space cell
                    dr, dc, didx = cell_from_xy(fcx, cy, w, h, GRID_R, GRID_C)
                    display_detect_cells.add(didx)

                    current_centers.append((fcx, cy))

            # ---------------- Per-cell timing logic ----------------
            output_mask = 0

            for idx in range(N_CELLS):
                is_active = now < active_until[idx]
                is_detected = idx in detected_cells

                if is_active:
                    # keep actuator on
                    output_mask |= (1 << idx)

                    # while active, ignore detection in same box
                    detect_start[idx] = None

                else:
                    # not active anymore
                    active_until[idx] = 0.0

                    if is_detected:
                        if detect_start[idx] is None:
                            detect_start[idx] = now
                        elif now - detect_start[idx] >= args.hold_detect_s:
                            active_until[idx] = now + args.actuate_hold_s
                            output_mask |= (1 << idx)
                            detect_start[idx] = None
                    else:
                        detect_start[idx] = None

            # ---------------- Display highlights ----------------
            # Show only boxes that are currently being detected
            # but do NOT show boxes that are already active
            for didx in display_detect_cells:
                # convert display cell index -> actuator cell index
                dr = didx // GRID_C
                dc = didx % GRID_C

                # map display row back to actuator row logic
                # display is mirrored horizontally only, but actuator logic is vertically flipped
                # for visual hiding, we only need to hide if the corresponding actuator cell is active
                actuator_r = GRID_R - 1 - dr
                actuator_c = dc
                actuator_idx = actuator_r * GRID_C + actuator_c

                if now >= active_until[actuator_idx]:
                    highlight_cell(disp, dr, dc, GRID_R, GRID_C)

            # ---------------- Trail logic ----------------
            if current_centers:
                avg_x = int(sum(p[0] for p in current_centers) / len(current_centers))
                avg_y = int(sum(p[1] for p in current_centers) / len(current_centers))
                trail_points.append((avg_x, avg_y, now))

            trail_points = [p for p in trail_points if now - p[2] <= args.trail_time_s]

            if len(trail_points) > args.trail_max_points:
                trail_points = trail_points[-args.trail_max_points:]

            for i in range(1, len(trail_points)):
                x1, y1, t1 = trail_points[i - 1]
                x2, y2, t2 = trail_points[i]

                age = now - t2
                if age <= args.trail_time_s:
                    frac = max(0.0, 1.0 - age / args.trail_time_s)
                    thickness = max(1, int(1 + 4 * frac))
                    cv2.line(disp, (x1, y1), (x2, y2), 255, thickness)

            # ---------------- Serial send ----------------
            if ser and output_mask != last_sent_mask:
                ser.write(f"{output_mask}\n".encode())
                last_sent_mask = output_mask
                print("Sent:", output_mask, "Cells:", mask_to_indices(output_mask, N_CELLS))

            # ---------------- Status text ----------------
            detecting_cells = []
            active_cells = []

            for idx in range(N_CELLS):
                if now < active_until[idx]:
                    active_cells.append(idx)
                elif detect_start[idx] is not None:
                    detecting_cells.append(idx)

            cv2.putText(
                disp,
                f"Active:{active_cells}",
                (10, 25),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.6,
                255
            )

            cv2.putText(
                disp,
                f"Detecting:{detecting_cells}",
                (10, 55),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.6,
                255
            )

            cv2.putText(
                disp,
                f"Mask:{output_mask}  Cells:{mask_to_indices(output_mask, N_CELLS)}",
                (10, 85),
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