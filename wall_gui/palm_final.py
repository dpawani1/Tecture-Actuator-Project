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

    # rectangular video now
    parser.add_argument("--video_width", type=int, default=960)
    parser.add_argument("--video_height", type=int, default=640)
    parser.add_argument("--fps", type=int, default=30)

    # timing
    parser.add_argument("--hold_detect_s", type=float, default=2.0)
    parser.add_argument("--actuate_hold_s", type=float, default=4.0)

    # disappearing center trail
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
    cam.setIspScale(2, 3)  # 720p from 1080p

    # rectangular video instead of square
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

    # ---------------- State ----------------
    palmDetection = PalmDetection()

    # detection must stay same for 2 sec before actuation
    candidate_mask = 0
    candidate_start_time = None

    # once active, stay active for 4 sec even if hand leaves
    active_mask = 0
    active_until = 0.0
    last_sent_mask = None

    # line trail points: (x, y, t)
    trail_points = []

    # ---------------- Window ----------------
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1200, 800)

    # ---------------- Run ----------------
    with dai.Device(pipeline) as device:
        vidQ = device.getOutputQueue("cam", 4, False)
        palmQ = device.getOutputQueue("palm_nn", 4, False)

        while True:
            now = time.time()

            frame = vidQ.get().getCvFrame()
            h, w = frame.shape[:2]

            # mirrored display only
            disp = cv2.flip(frame, 1)
            draw_grid(disp, GRID_R, GRID_C)

            detected_mask = 0
            display_cells = []
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

                    # actuator mapping:
                    # vertical flip only, because that matched your hardware
                    r, c, _ = cell_from_xy(cx, cy, w, h, GRID_R, GRID_C)
                    mapped_r = GRID_R - 1 - r
                    idx = mapped_r * GRID_C + c
                    detected_mask |= (1 << idx)

                    # mirrored display drawing
                    fx1 = w - x2
                    fx2 = w - x1
                    fcx = w - cx

                    cv2.rectangle(disp, (fx1, y1), (fx2, y2), 255, 2)
                    cv2.circle(disp, (fcx, cy), 4, 255, -1)

                    # display highlight cell
                    dr, dc, _ = cell_from_xy(fcx, cy, w, h, GRID_R, GRID_C)
                    display_cells.append((dr, dc))

                    # save center for trail
                    current_centers.append((fcx, cy))

                for dr, dc in display_cells:
                    highlight_cell(disp, dr, dc, GRID_R, GRID_C)

            # ---------------- Detection delay logic ----------------
            # If something is already active, ignore new triggers until timer ends
            if now < active_until:
                output_mask = active_mask
            else:
                # active period ended
                if active_mask != 0:
                    active_mask = 0

                # need same nonzero mask for 2 seconds before firing
                if detected_mask != 0:
                    if detected_mask != candidate_mask:
                        candidate_mask = detected_mask
                        candidate_start_time = now
                    else:
                        if candidate_start_time is not None and (now - candidate_start_time) >= args.hold_detect_s:
                            active_mask = candidate_mask
                            active_until = now + args.actuate_hold_s
                            output_mask = active_mask

                            # clear candidate once it fires
                            candidate_mask = 0
                            candidate_start_time = None
                        else:
                            output_mask = 0
                else:
                    candidate_mask = 0
                    candidate_start_time = None
                    output_mask = 0

                if now < active_until:
                    output_mask = active_mask

            # ---------------- Trail logic ----------------
            # Only add trail while detecting and not during locked active period
            if now >= active_until and current_centers:
                avg_x = int(sum(p[0] for p in current_centers) / len(current_centers))
                avg_y = int(sum(p[1] for p in current_centers) / len(current_centers))
                trail_points.append((avg_x, avg_y, now))

            # remove old trail points
            trail_points = [p for p in trail_points if now - p[2] <= args.trail_time_s]

            # keep list from growing too large
            if len(trail_points) > args.trail_max_points:
                trail_points = trail_points[-args.trail_max_points:]

            # draw disappearing trail
            for i in range(1, len(trail_points)):
                x1, y1, t1 = trail_points[i - 1]
                x2, y2, t2 = trail_points[i]

                age = now - t2
                if age <= args.trail_time_s:
                    # newer line thicker, older line thinner
                    frac = max(0.0, 1.0 - age / args.trail_time_s)
                    thickness = max(1, int(1 + 4 * frac))
                    cv2.line(disp, (x1, y1), (x2, y2), 255, thickness)

            # ---------------- Serial send ----------------
            if ser and output_mask != last_sent_mask:
                ser.write(f"{output_mask}\n".encode())
                last_sent_mask = output_mask
                print("Sent:", output_mask, "Cells:", mask_to_indices(output_mask, N_CELLS))

            # ---------------- Status text ----------------
            if now < active_until:
                status = f"ACTIVE {max(0.0, active_until - now):.1f}s"
            elif candidate_mask != 0 and candidate_start_time is not None:
                status = f"DETECTING {max(0.0, args.hold_detect_s - (now - candidate_start_time)):.1f}s"
            else:
                status = "IDLE"

            cv2.putText(
                disp,
                f"Status:{status}",
                (10, 25),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.6,
                255
            )

            cv2.putText(
                disp,
                f"Mask:{output_mask}  Cells:{mask_to_indices(output_mask, N_CELLS)}",
                (10, 55),
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