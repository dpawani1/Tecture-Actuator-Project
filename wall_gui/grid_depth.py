import cv2
import depthai as dai
import numpy as np
import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 9600

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
time.sleep(2)

TRIGGER_MM = 1500
K_ON = 3
HOLD_TIME_S = 2.0

below_cnt = [0] * 9
active = [False] * 9
hold_until = [0.0] * 9

WINDOW_NAME = "3x3 Depth Grid"

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
roi_margin = 0.05

for r in range(gridN):
    for c in range(gridN):
        cell_x0 = c / gridN
        cell_y0 = r / gridN
        cell_x1 = (c + 1) / gridN
        cell_y1 = (r + 1) / gridN

        dx = (cell_x1 - cell_x0) * roi_margin
        dy = (cell_y1 - cell_y0) * roi_margin

        topLeft = dai.Point2f(cell_x0 + dx, cell_y0 + dy)
        bottomRight = dai.Point2f(cell_x1 - dx, cell_y1 - dy)

        cfg = dai.SpatialLocationCalculatorConfigData()
        cfg.depthThresholds.lowerThreshold = 100
        cfg.depthThresholds.upperThreshold = 10000
        cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN
        cfg.roi = dai.Rect(topLeft, bottomRight)

        spatialLocationCalculator.initialConfig.addROI(cfg)

spatialLocationCalculator.inputConfig.setWaitForMessage(False)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(spatialLocationCalculator.inputDepth)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
spatialLocationCalculator.out.link(xoutSpatialData.input)

last_sent_mask = None

with dai.Device(pipeline) as device:
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1100, 900)

    color_idle = (255, 255, 255)
    color_active = (0, 255, 0)

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

        for i in range(min(9, len(spatialData))):
            z = int(spatialData[i].spatialCoordinates.z)
            detected = (z > 0) and (z <= TRIGGER_MM)

            if active[i]:
                if now < hold_until[i]:
                    pass
                else:
                    if detected:
                        hold_until[i] = now + HOLD_TIME_S
                    else:
                        active[i] = False
                        below_cnt[i] = 0
                        hold_until[i] = 0.0
            else:
                if detected:
                    below_cnt[i] += 1
                else:
                    below_cnt[i] = 0

                if below_cnt[i] >= K_ON:
                    active[i] = True
                    below_cnt[i] = 0
                    hold_until[i] = now + HOLD_TIME_S

        mask = 0
        for i in range(9):
            if active[i]:
                mask |= (1 << i)

        if mask != last_sent_mask:
            ser.write(f"{mask}\n".encode("ascii"))
            last_sent_mask = mask

        depthColor = cv2.flip(depthColor, 1)

        h, w = depthColor.shape[:2]

        for i, depthData in enumerate(spatialData):
            roi = depthData.config.roi
            roi = roi.denormalize(width=w, height=h)

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            flipped_xmin = w - xmax
            flipped_xmax = w - xmin

            draw_color = color_active if active[i] else color_idle

            cv2.rectangle(
                depthColor,
                (flipped_xmin, ymin),
                (flipped_xmax, ymax),
                draw_color,
                2
            )

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
            f"Threshold: {TRIGGER_MM} mm",
            (10, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color_idle,
            1
        )

        cv2.imshow(WINDOW_NAME, depthColor)

        if cv2.waitKey(1) == ord('q'):
            break

ser.close()
cv2.destroyAllWindows()