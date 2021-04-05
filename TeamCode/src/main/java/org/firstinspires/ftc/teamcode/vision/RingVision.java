package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingVision {
    HardwareMap hardwareMap;

    OpenCvCamera webcam;
    RingVisionPipeline pipeline;

    public enum RingCount { FOUR, ONE, ZERO }
    public enum TargetZone { ZONE_A, ZONE_B, ZONE_C }

    private static final Scalar BLUE = new Scalar(0, 0, 255);
    private static final Scalar GREEN = new Scalar(0, 255, 0);
    private static final Scalar RED = new Scalar(255, 0, 0);

    private int _thresholdRingsFour = 0;
    private int _thresholdRingsOne = 0;
    private Point _regionTopLeft = new Point(0,0);
    private Point _regionBottomRight = new Point(0,0);

    public RingVision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init(AutoUtils.Alliance alliance, AutoUtils.StartingPosition startingPosition) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingVisionPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(() ->
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );

        int boxHeight = 0;
        int boxWidth = 0;
        int boxX = 0;
        int boxY = 0;

        //--- Red - Right Starting (Far Right of Line)
        if (alliance == AutoUtils.Alliance.RED && startingPosition == AutoUtils.StartingPosition.OUTSIDE) {
            _thresholdRingsFour = 138;
            _thresholdRingsOne = 130;
            boxX = 10;
            boxY = 116;
            boxWidth = 26;
            boxHeight = 20;
        }
        //--- Red - Left Starting (Far Left of Line)
        else if (alliance == AutoUtils.Alliance.RED && startingPosition == AutoUtils.StartingPosition.INSIDE) {
            _thresholdRingsFour = 138;
            _thresholdRingsOne = 130;
            boxX = 235;
            boxY = 126;
            boxWidth = 28;
            boxHeight = 20;
        }
        //--- Blue - Right Starting (Far Right of Line)
        else if (alliance == AutoUtils.Alliance.BLUE && startingPosition == AutoUtils.StartingPosition.INSIDE) {
            _thresholdRingsFour = 138;
            _thresholdRingsOne = 130;
            boxX = 10;
            boxY = 116;
            boxWidth = 26;
            boxHeight = 20;
        }
        //--- Blue - Left Starting (Center of Line)
        else if (alliance == AutoUtils.Alliance.BLUE && startingPosition == AutoUtils.StartingPosition.OUTSIDE) {
            _thresholdRingsFour = 138;
            _thresholdRingsOne = 130;
            boxX = 235;
            boxY = 126;
            boxWidth = 28;
            boxHeight = 20;
        }

        _regionTopLeft = new Point(boxX, boxY);
        _regionBottomRight = new Point(boxX + boxWidth, boxY + boxHeight);

        setViewportPaused(false);
    }

    private class RingVisionPipeline extends OpenCvPipeline {
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        private boolean viewportPaused;
        private int averageColorLevel;
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingCount ringCount = RingCount.FOUR;

        private List<RingsFound> regionsToSearch = new ArrayList<>();

        /*
        Takes the RGB, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);

            regionsToSearch.add(AddRegionToSearch(0, 0));


        }

        private RingsFound AddRegionToSearch(int offsetX, int offsetY)
        {
            RingsFound ringsFound = new RingsFound();
            ringsFound.topLeft = new Point(_regionTopLeft.x + offsetX, _regionTopLeft.y + offsetY);
            ringsFound.bottomRight = new Point(_regionBottomRight.x + offsetX, _regionBottomRight.y + offsetY);
            return ringsFound;
        }

        void drawDetectionPreview(Mat input, RingCount count, int colorLevel, Point regionTopLeft, Point regionBottomRight) {

            // Draw box
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    regionTopLeft, // First point which defines the rectangle
                    regionBottomRight, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            // Draw target
            int lineLength = 50;
            Imgproc.line(input,
                    new Point(regionTopLeft.x + Math.round((regionBottomRight.x - regionTopLeft.x) / 2), regionTopLeft.y - lineLength + 2),
                    new Point(regionTopLeft.x + Math.round((regionBottomRight.x - regionTopLeft.x) / 2), regionTopLeft.y - 2),
                    RED, 2);
            Imgproc.line(input,
                    new Point(regionTopLeft.x + Math.round((regionBottomRight.x - regionTopLeft.x) / 2), regionBottomRight.y + lineLength + 2),
                    new Point(regionTopLeft.x + Math.round((regionBottomRight.x - regionTopLeft.x) / 2), regionBottomRight.y + 2),
                    RED, 2);
            Imgproc.line(input,
                    new Point(regionTopLeft.x - lineLength + 2, regionTopLeft.y + Math.round((regionBottomRight.y - regionTopLeft.y) / 2)),
                    new Point(regionTopLeft.x - 2, regionTopLeft.y + Math.round((regionBottomRight.y - regionTopLeft.y) / 2)),
                    RED, 2);
            Imgproc.line(input,
                    new Point(regionBottomRight.x + lineLength + 2, regionTopLeft.y + Math.round((regionBottomRight.y - regionTopLeft.y) / 2)),
                    new Point(regionBottomRight.x + 2, regionTopLeft.y + Math.round((regionBottomRight.y - regionTopLeft.y) / 2)),
                    RED, 2);

            // Draw text
            Scalar displayColor = RED;
            if (count == RingCount.FOUR) {
                displayColor = GREEN;
            } else if (count == RingCount.ONE) {
                displayColor = BLUE;
            }
            Imgproc.putText(input, count + " RING(S)", new Point(75, 190), 1, 2, displayColor, 2);
            Imgproc.putText(input, String.valueOf(colorLevel), new Point(135, 230), 1, 2, displayColor, 2);
        }

        @Override
        public Mat processFrame(Mat input) {
            // Calculate color level
            inputToCb(input);

            RingsFound found = new RingsFound();
            found = GetRingCountFromCoordinates(_regionTopLeft, _regionBottomRight);

            averageColorLevel = found.colorLevel;
            ringCount = found.count;

            drawDetectionPreview(input, found.count, found.colorLevel, found.topLeft, found.bottomRight);

            return input;
        }

        private RingsFound GetRingCountFromCoordinates(Point topLeft, Point bottomRight)
        {
            Mat region = Cb.submat(new Rect(topLeft, bottomRight));
            RingsFound found = GetRingCountForRegion(region);
            found.topLeft = topLeft;
            found.bottomRight = bottomRight;
            return found;
        }

        private RingsFound GetRingCountForRegion(Mat region)
        {
            RingsFound found = new RingsFound();
            found.colorLevel = (int) Core.mean(region).val[0];
            found.count = GetRingCountFromColorLevel(found.colorLevel);
            return found;
        }

        private RingCount GetRingCountFromColorLevel(int level)
        {
            RingCount count;
            if (level > _thresholdRingsFour) {
                count = RingCount.FOUR;
            } else if (level > _thresholdRingsOne) {
                count = RingCount.ONE;
            } else {
                count = RingCount.ZERO;
            }
            return count;
        }

        private class RingsFound {
            private RingCount count = RingCount.FOUR;
            private int colorLevel = 0;
            private Point topLeft = new Point(0,0);
            private Point bottomRight = new Point(0,0);
        }

        @Override
        public void onViewportTapped() {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            setViewportPaused(!viewportPaused);
        }

        public void setViewportPaused(boolean paused) {
            viewportPaused = paused;

            if(viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }


    public void setViewportPaused(boolean paused) {
        pipeline.setViewportPaused(paused);
    }

    public boolean isViewportPaused() {
        return pipeline.viewportPaused;
    }

    public int getColorLevel() {
        return pipeline.averageColorLevel;
    }

    public RingCount getRingCount() {
        return pipeline.ringCount;
    }

    public TargetZone getTargetZone() {
        switch (getRingCount()) {
            case ONE:
                return TargetZone.ZONE_B;
            case FOUR:
                return TargetZone.ZONE_C;
            default:
                return TargetZone.ZONE_A;
        }
    }
}
