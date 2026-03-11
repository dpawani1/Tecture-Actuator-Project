import cv2
import depthai as dai
import numpy as np
import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 9600

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
time.sleep(2)  # Arduino resets on connect

TRIGGER_MM = 1500

# Require a few frames to confirm detection before turning on
K_ON = 3
below_cnt = [0] * 9

# Active state per cell
active = [False] * 9

# Hold each active cell for at least 2 seconds before checking again
HOLD_TIME_S = 2.0
hold_until = [0.0] * 9

pipeline = dai.Pipeline()

monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutSpatialData = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

gridN = 3
for r in range(gridN):
    for c in range(gridN):
        topLeft = dai.Point2f(c / gridN, r / gridN)
        bottomRight = dai.Point2f((c + 1) / gridN, (r + 1) / gridN)

        cfg = dai.SpatialLocationCalculatorConfigData()
        cfg.depthThresholds.lowerThreshold = 100
        cfg.depthThresholds.upperThreshold = 10000
        cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        cfg.roi = dai.Rect(topLeft, bottomRight)

        spatialLocationCalculator.initialConfig.addROI(cfg)

spatialLocationCalculator.inputConfig.setWaitForMessage(False)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(spatialLocationCalculator.inputDepth)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
spatialLocationCalculator.out.link(xoutSpatialData.input)

last_sent_mask = None

WINDOW_NAME = "3x3 Depth Grid"
DISPLAY_SCALE = 1.8  # bigger preview

with dai.Device(pipeline) as device:
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)

    color = (255, 255, 255)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    while True:
        inDepth = depthQueue.get()
        depthFrame = inDepth.getFrame()

        depth_downscaled = depthFrame[::4]
        nonzero = depth_downscaled[depth_downscaled != 0]

        if nonzero.size == 0:
            min_depth = 0
            max_depth = 1
        else:
            min_depth = np.percentile(nonzero, 1)
            max_depth = np.percentile(nonzero, 99)
            if max_depth <= min_depth:
                max_depth = min_depth + 1

        depthColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthColor = cv2.applyColorMap(depthColor, cv2.COLORMAP_HOT)

        spatialData = spatialQueue.get().getSpatialLocations()
        now = time.time()

        # Update active[] with:
        # - 3-frame debounce for turning on
        # - 2 second hold before re-checking
        for i in range(min(9, len(spatialData))):
            z = int(spatialData[i].spatialCoordinates.z)

            detected = (z > 0) and (z <= TRIGGER_MM)

            if active[i]:
                # While holding, ignore detection changes
                if now < hold_until[i]:
                    pass
                else:
                    # Re-check after hold expires
                    if detected:
                        hold_until[i] = now + HOLD_TIME_S
                    else:
                        active[i] = False
                        below_cnt[i] = 0
                        hold_until[i] = 0.0
            else:
                # Inactive: require K_ON consecutive detections
                if detected:
                    below_cnt[i] += 1
                else:
                    below_cnt[i] = 0

                if below_cnt[i] >= K_ON:
                    active[i] = True
                    below_cnt[i] = 0
                    hold_until[i] = now + HOLD_TIME_S

        # Build mask
        mask = 0
        for i in range(9):
            if active[i]:
                mask |= (1 << i)

        # Send only when changed
        if mask != last_sent_mask:
            ser.write(f"{mask}\n".encode("ascii"))
            last_sent_mask = mask

        # Flip preview horizontally to match your palm-tracking orientation
        depthColor = cv2.flip(depthColor, 1)

        # Draw grid + Z text
        h, w = depthColor.shape[:2]

        for i, depthData in enumerate(spatialData):
            roi = depthData.config.roi
            roi = roi.denormalize(width=w, height=h)

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            # Because preview is flipped, mirror the x coordinates too
            flipped_xmin = w - xmax
            flipped_xmax = w - xmin

            draw_color = (0, 255, 0) if active[i] else color

            cv2.rectangle(depthColor, (flipped_xmin, ymin), (flipped_xmax, ymax), draw_color, 2)

            ztxt = int(depthData.spatialCoordinates.z)
            label = f"Z: {ztxt} mm"
            if active[i]:
                label += " ON"

            cv2.putText(
                depthColor,
                label,
                (flipped_xmin + 5, ymin + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                draw_color,
                1
            )

        cv2.putText(
            depthColor,
            f"MASK: {mask}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

        cv2.putText(
            depthColor,
            f"THRESHOLD: {TRIGGER_MM} mm",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

        # Bigger preview
        displayFrame = cv2.resize(
            depthColor,
            None,
            fx=DISPLAY_SCALE,
            fy=DISPLAY_SCALE,
            interpolation=cv2.INTER_LINEAR
        )

        cv2.imshow(WINDOW_NAME, displayFrame)

        if cv2.waitKey(1) == ord('q'):
            break

ser.close()
cv2.destroyAllWindows()