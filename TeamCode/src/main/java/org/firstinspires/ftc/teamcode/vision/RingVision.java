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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class RingVision {
    HardwareMap hardwareMap;

    OpenCvCamera webcam;
    RingVisionPipeline pipeline;

    public enum RingCount { FOUR, ONE, ZERO }

    private static Scalar _blue = new Scalar(0, 0, 255);
    private static Scalar _green = new Scalar(0, 255, 0);
    private static Scalar _red = new Scalar(255, 0, 0);
    private static AutoUtils.Alliance _alliance;
    private static AutoUtils.StartingPosition _startingPosition;

    private int _thresholdRingsFour = 0;
    private int _thresholdRingsOne = 0;
    private int _boxHeight = 0;
    private int _boxWidth = 0;
    private int _boxX = 0;
    private int _boxY = 0;

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

        _startingPosition = startingPosition;

        //--- Red - Right Starting (Far Right of Line)
        if (_alliance == AutoUtils.Alliance.RED && _startingPosition == AutoUtils.StartingPosition.OUTSIDE) {
            _thresholdRingsFour = 138;
            _thresholdRingsOne = 130;
            _boxX = 10;
            _boxY = 116;
            _boxWidth = 26;
            _boxHeight = 20;
        }
        //--- Red - Left Starting (Far Left of Line)
        else if (_alliance == AutoUtils.Alliance.RED && _startingPosition == AutoUtils.StartingPosition.INSIDE) {
            _thresholdRingsFour = 138;
            _thresholdRingsOne = 130;
            _boxX = 235;
            _boxY = 126;
            _boxWidth = 28;
            _boxHeight = 20;
        }
        //--- Blue - Right Starting (Far Right of Line)
        else if (_alliance == AutoUtils.Alliance.BLUE && _startingPosition == AutoUtils.StartingPosition.INSIDE) {
            _thresholdRingsFour = 138;
            _thresholdRingsOne = 130;
            _boxX = 10;
            _boxY = 116;
            _boxWidth = 26;
            _boxHeight = 20;
        }
        //--- Blue - Left Starting (Center of Line)
        else if (_alliance == AutoUtils.Alliance.BLUE && _startingPosition == AutoUtils.StartingPosition.OUTSIDE) {
            _thresholdRingsFour = 138;
            _thresholdRingsOne = 130;
            _boxX = 235;
            _boxY = 126;
            _boxWidth = 28;
            _boxHeight = 20;
        }

        _regionTopLeft = new Point(_boxX, _boxY);
        _regionBottomRight = new Point(_boxX+_boxWidth,_boxY+_boxHeight);
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

    private class RingVisionPipeline extends OpenCvPipeline {
        Mat regionCb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        private boolean viewportPaused;
        private int averageColorLevel;
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingCount ringCount = RingCount.FOUR;

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
            regionCb = Cb.submat(new Rect(_regionTopLeft, _regionBottomRight));
        }

        void drawDetectionPreview(Mat input) {

            // Draw box
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    _regionTopLeft, // First point which defines the rectangle
                    _regionBottomRight, // Second point which defines the rectangle
                    _blue, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            // Draw target
            int lineLength = 50;
            Imgproc.line(input,
                    new Point(_regionTopLeft.x + Math.round((_regionBottomRight.x - _regionTopLeft.x) / 2), _regionTopLeft.y - lineLength + 2),
                    new Point(_regionTopLeft.x + Math.round((_regionBottomRight.x - _regionTopLeft.x) / 2),_regionTopLeft.y - 2),
                    _red, 2);
            Imgproc.line(input,
                    new Point(_regionTopLeft.x + Math.round((_regionBottomRight.x - _regionTopLeft.x) / 2), _regionBottomRight.y + lineLength + 2),
                    new Point(_regionTopLeft.x + Math.round((_regionBottomRight.x - _regionTopLeft.x) / 2),_regionBottomRight.y + 2),
                    _red, 2);
            Imgproc.line(input,
                    new Point(_regionTopLeft.x - lineLength + 2, _regionTopLeft.y + Math.round((_regionBottomRight.y - _regionTopLeft.y) / 2)),
                    new Point(_regionTopLeft.x - 2, _regionTopLeft.y + Math.round((_regionBottomRight.y - _regionTopLeft.y) / 2)),
                    _red, 2);
            Imgproc.line(input,
                    new Point(_regionBottomRight.x + lineLength + 2, _regionTopLeft.y + Math.round((_regionBottomRight.y - _regionTopLeft.y) / 2)),
                    new Point(_regionBottomRight.x + 2, _regionTopLeft.y + Math.round((_regionBottomRight.y - _regionTopLeft.y) / 2)),
                    _red, 2);

            // Draw text
            Scalar displayColor = _red;
            if (pipeline.ringCount == RingCount.FOUR) {
                displayColor = _green;
            } else if (pipeline.ringCount == RingCount.ONE) {
                displayColor = _blue;
            }
            Imgproc.putText(input, ringCount + " RING(S)", new Point(75, 190), 1, 2, displayColor, 2);
            Imgproc.putText(input, String.valueOf(averageColorLevel), new Point(135, 230), 1, 2, displayColor, 2);
        }

        @Override
        public Mat processFrame(Mat input) {
            // Calculate color level
            inputToCb(input);
            averageColorLevel = (int) Core.mean(regionCb).val[0];

            ringCount = RingCount.FOUR;
            if (averageColorLevel > _thresholdRingsFour) {
                ringCount = RingCount.FOUR;
            } else if (averageColorLevel > _thresholdRingsOne) {
                ringCount = RingCount.ONE;
            } else {
                ringCount = RingCount.ZERO;
            }

            drawDetectionPreview(input);

            return input;
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
}
