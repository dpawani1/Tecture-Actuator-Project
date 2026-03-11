#!/usr/bin/env python3

import time
import math
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


def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def color_for_id(track_id):
    palette = [
        (255, 255, 255),  # white
        (255, 255, 0),    # cyan
        (0, 255, 255),    # yellow
        (255, 0, 255),    # magenta
        (0, 255, 0),      # green
        (0, 128, 255),    # orange
        (255, 0, 0),      # blue
        (0, 0, 255),      # red
        (128, 0, 255),    # purple
    ]
    return palette[track_id % len(palette)]


def update_tracks(tracks, detections, now, next_track_id,
                  max_match_dist=100, max_track_age=0.7, trail_time_s=2.0, trail_max_points=120):
    matched_tracks = set()
    matched_dets = set()

    track_ids = list(tracks.keys())

    # greedy nearest-neighbor matching
    pairs = []
    for tid in track_ids:
        last_pt = tracks[tid]["last_pos"]
        for di, det in enumerate(detections):
            pairs.append((dist(last_pt, det), tid, di))
    pairs.sort(key=lambda x: x[0])

    for d, tid, di in pairs:
        if d > max_match_dist:
            continue
        if tid in matched_tracks or di in matched_dets:
            continue

        matched_tracks.add(tid)
        matched_dets.add(di)

        tracks[tid]["last_pos"] = detections[di]
        tracks[tid]["last_seen"] = now
        tracks[tid]["points"].append((detections[di][0], detections[di][1], now))
        if len(tracks[tid]["points"]) > trail_max_points:
            tracks[tid]["points"] = tracks[tid]["points"][-trail_max_points:]

    # new tracks for unmatched detections
    for di, det in enumerate(detections):
        if di in matched_dets:
            continue
        tid = next_track_id
        next_track_id += 1
        tracks[tid] = {
            "id": tid,
            "color": color_for_id(tid),
            "last_pos": det,
            "last_seen": now,
            "points": [(det[0], det[1], now)],
        }

    # prune old points and dead tracks
    dead_ids = []
    for tid, tr in tracks.items():
        tr["points"] = [p for p in tr["points"] if now - p[2] <= trail_time_s]
        if (now - tr["last_seen"] > max_track_age) and not tr["points"]:
            dead_ids.append(tid)

    for tid in dead_ids:
        del tracks[tid]

    return next_track_id


def draw_tracks(frame, tracks, now, trail_time_s):
    for tid, tr in tracks.items():
        pts = tr["points"]
        color = tr["color"]

        for i in range(1, len(pts)):
            x1, y1, t1 = pts[i - 1]
            x2, y2, t2 = pts[i]

            age = now - t2
            if age <= trail_time_s:
                frac = max(0.0, 1.0 - age / trail_time_s)
                thickness = max(1, int(1 + 4 * frac))
                cv2.line(frame, (x1, y1), (x2, y2), color, thickness)

        if pts:
            x, y, t = pts[-1]
            if now - t <= trail_time_s:
                cv2.circle(frame, (x, y), 5, color, -1)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--grid_rows", type=int, default=3)
    parser.add_argument("--grid_cols", type=int, default=3)
    parser.add_argument("--max_palms", type=int, default=6)

    parser.add_argument("--serial_port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=9600)

    # back to square
    parser.add_argument("--video_size", type=int, default=640)
    parser.add_argument("--fps", type=int, default=50)

    parser.add_argument("--hold_detect_s", type=float, default=1.0)
    parser.add_argument("--actuate_hold_s", type=float, default=6.0)

    parser.add_argument("--trail_time_s", type=float, default=2.0)
    parser.add_argument("--trail_max_points", type=int, default=120)
    parser.add_argument("--track_match_dist", type=float, default=100.0)
    parser.add_argument("--track_max_age", type=float, default=0.7)

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

    palmDetection = PalmDetection()

    # per-cell timers
    detect_start = [None] * N_CELLS
    active_until = [0.0] * N_CELLS

    # multi-hand tracks
    tracks = {}
    next_track_id = 0

    last_sent_mask = None

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1000, 1000)

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

            detected_cells = set()
            display_detect_cells = set()
            display_centers = []

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

                    # actuator mapping
                    r, c, _ = cell_from_xy(cx, cy, w, h, GRID_R, GRID_C)
                    mapped_r = GRID_R - 1 - r
                    idx = mapped_r * GRID_C + c
                    detected_cells.add(idx)

                    # display mapping
                    fx1 = w - x2
                    fx2 = w - x1
                    fcx = w - cx

                    cv2.rectangle(disp, (fx1, y1), (fx2, y2), 255, 2)
                    cv2.circle(disp, (fcx, cy), 4, 255, -1)

                    dr, dc, didx = cell_from_xy(fcx, cy, w, h, GRID_R, GRID_C)
                    display_detect_cells.add(didx)
                    display_centers.append((fcx, cy))

            # ---------------- Multi-hand tracking ----------------
            next_track_id = update_tracks(
                tracks,
                display_centers,
                now,
                next_track_id,
                max_match_dist=args.track_match_dist,
                max_track_age=args.track_max_age,
                trail_time_s=args.trail_time_s,
                trail_max_points=args.trail_max_points
            )

            # ---------------- Per-cell timing logic ----------------
            output_mask = 0

            for idx in range(N_CELLS):
                is_detected = idx in detected_cells
                is_active = now < active_until[idx]

                if is_detected:
                    if detect_start[idx] is None:
                        detect_start[idx] = now

                    # activate after 2 sec
                    if (now - detect_start[idx]) >= args.hold_detect_s:
                        # keep extending while hand remains there
                        active_until[idx] = max(active_until[idx], now + args.actuate_hold_s)

                else:
                    detect_start[idx] = None

                if now < active_until[idx]:
                    output_mask |= (1 << idx)
                else:
                    active_until[idx] = 0.0

            # ---------------- Display highlights ----------------
            # Show detection highlights all the time, even for active cells
            for didx in display_detect_cells:
                dr = didx // GRID_C
                dc = didx % GRID_C
                highlight_cell(disp, dr, dc, GRID_R, GRID_C)

            # draw multi-hand colored trails
            draw_tracks(disp, tracks, now, args.trail_time_s)

            # ---------------- Serial send ----------------
            if ser and output_mask != last_sent_mask:
                ser.write(f"{output_mask}\n".encode())
                last_sent_mask = output_mask
                print("Sent:", output_mask, "Cells:", mask_to_indices(output_mask, N_CELLS))

            cv2.imshow(WIN, disp)

            if cv2.waitKey(1) == ord('q'):
                break

    if ser:
        ser.close()


if __name__ == "__main__":
    main()