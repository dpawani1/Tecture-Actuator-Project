import cv2
import depthai as dai
import numpy as np

# Create pipeline
pipeline = dai.Pipeline()

# Nodes
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutSpatialData = pipeline.create(dai.node.XLinkOut)

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")

# Camera settings
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")

monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

# Create 3x3 grid ROIs
gridN = 3
configs = []

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
        configs.append(cfg)

spatialLocationCalculator.inputConfig.setWaitForMessage(False)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(spatialLocationCalculator.inputDepth)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
spatialLocationCalculator.out.link(xoutSpatialData.input)

# Run
with dai.Device(pipeline) as device:

    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)

    color = (255, 255, 255)

    while True:
        inDepth = depthQueue.get()
        depthFrame = inDepth.getFrame()

        # Normalize depth for display
        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)

        max_depth = np.percentile(depth_downscaled, 99)

        depthColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthColor = cv2.applyColorMap(depthColor, cv2.COLORMAP_HOT)

        spatialData = spatialQueue.get().getSpatialLocations()

        for i, depthData in enumerate(spatialData):
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthColor.shape[1], height=depthColor.shape[0])

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            cv2.rectangle(depthColor, (xmin, ymin), (xmax, ymax), color, 1)

            cv2.putText(depthColor, f"Z: {int(depthData.spatialCoordinates.z)} mm",
                        (xmin + 5, ymin + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color)

        cv2.imshow("3x3 Depth Grid", depthColor)

        if cv2.waitKey(1) == ord('q'):
            break
