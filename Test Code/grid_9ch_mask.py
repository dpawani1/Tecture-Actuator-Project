import cv2
import depthai as dai
import numpy as np
import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 9600

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
time.sleep(2)  # Arduino resets on connect

TRIGGER_MM = 800
RESET_MM = 1600  # must be > TRIGGER_MM

# 3-frame debounce
K_ON = 3
K_OFF = 3
below_cnt = [0] * 9
above_cnt = [0] * 9
active = [False] * 9

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

with dai.Device(pipeline) as device:
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)

    color = (255, 255, 255)

    while True:
        inDepth = depthQueue.get()
        depthFrame = inDepth.getFrame()

        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)

        depthColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthColor = cv2.applyColorMap(depthColor, cv2.COLORMAP_HOT)

        spatialData = spatialQueue.get().getSpatialLocations()

        # Update active[] with 3-frame debounce + hysteresis
        for i in range(min(9, len(spatialData))):
            z = int(spatialData[i].spatialCoordinates.z)

            # Treat invalid as "far" to avoid random triggers
            if z <= 0:
                below_cnt[i] = 0
                above_cnt[i] += 1
            else:
                if z < TRIGGER_MM:
                    below_cnt[i] += 1
                    above_cnt[i] = 0
                elif z > RESET_MM:
                    above_cnt[i] += 1
                    below_cnt[i] = 0
                else:
                    # in the middle band (500..800): don't accumulate either direction
                    below_cnt[i] = 0
                    above_cnt[i] = 0

            if (not active[i]) and (below_cnt[i] >= K_ON):
                active[i] = True
                below_cnt[i] = 0
                above_cnt[i] = 0

            if active[i] and (above_cnt[i] >= K_OFF):
                active[i] = False
                below_cnt[i] = 0
                above_cnt[i] = 0

        # Build mask
        mask = 0
        for i in range(9):
            if active[i]:
                mask |= (1 << i)

        # Send only when changed
        if mask != last_sent_mask:
            ser.write(f"{mask}\n".encode("ascii"))
            last_sent_mask = mask

        # Draw grid + Z text
        for i, depthData in enumerate(spatialData):
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthColor.shape[1], height=depthColor.shape[0])

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            cv2.rectangle(depthColor, (xmin, ymin), (xmax, ymax), color, 1)

            ztxt = int(depthData.spatialCoordinates.z)
            cv2.putText(depthColor, f"Z: {ztxt} mm",
                        (xmin + 5, ymin + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color)

        cv2.putText(depthColor, f"MASK: {mask}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

        cv2.imshow("3x3 Depth Grid", depthColor)

        if cv2.waitKey(1) == ord('q'):
            break

ser.close()
cv2.destroyAllWindows()
