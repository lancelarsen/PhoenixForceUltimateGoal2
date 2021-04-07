package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.appendages.BlinkinPatterns;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.vision.RingVision;

abstract public class AbstractAuto extends LinearOpMode {
    public MecanumAutonomous drive;
    public volatile AppendagesAutonomous appendages;
    public volatile RingVision ringVision;

    private static final long LIGHT_LONG_FLASH_ON_TIME = 1500;
    private static final long LIGHT_SHORT_FLASH_ON_TIME = 500;
    private static final long LIGHT_FLASH_OFF_TIME = 250;

    private enum FlashLength {
        LONG,
        SHORT
    }

    public void initAuto(AutoUtils.Alliance alliance, AutoUtils.StartingPosition startingPosition) {
        drive = new MecanumAutonomous(this);
        appendages = new AppendagesAutonomous(this);
        ringVision = new RingVision(hardwareMap);

        drive.setSpeed(MecanumAutonomous.Speed.FAST);

        ringVision.init(alliance, startingPosition);

        if (alliance == AutoUtils.Alliance.BLUE) {
            if (startingPosition == AutoUtils.StartingPosition.OUTSIDE) {
                drive.setCurrentPosition(FieldPositions.S1);
            } else {
                drive.setCurrentPosition(FieldPositions.S2);
            }

            updateLights(BlinkinPatterns.BLUE_BASE_PATTERN);
        } else {
            if (startingPosition == AutoUtils.StartingPosition.INSIDE) {
                drive.setCurrentPosition(FieldPositions.S3);
            } else {
                drive.setCurrentPosition(FieldPositions.S4);
            }

            updateLights(BlinkinPatterns.RED_BASE_PATTERN);
        }

        if (isStopRequested()) return;

        ringVision.setViewportPaused(true);
        lightsOff();
    }

    // Runs till opmode start
    public void updateLights(RevBlinkinLedDriver.BlinkinPattern basePattern) {
        Runnable lightTask = () -> {
            while (!Thread.interrupted()) {
                flashLights(basePattern, FlashLength.LONG);

                int ringCount;
                switch (ringVision.getRingCount()) {
                    case ONE:
                        ringCount = 1;
                        break;
                    case FOUR:
                        ringCount = 4;
                        break;
                    default:
                        ringCount = 0;
                        break;
                }

                for (int i = 0; i < ringCount; i++) {
                    flashLights(BlinkinPatterns.RING_PATTERN, FlashLength.SHORT);
                }
            }
        };

        Thread lightThread = new Thread(lightTask);
        lightThread.start();

        while (!isStarted());
        lightThread.interrupt();

        lightsOff();
    }

    private void flashLights(RevBlinkinLedDriver.BlinkinPattern pattern, FlashLength length) {
        appendages.setBlinkinPattern(pattern);
        sleep(length == FlashLength.LONG ? LIGHT_LONG_FLASH_ON_TIME : LIGHT_SHORT_FLASH_ON_TIME);
        lightsOff();
        sleep(LIGHT_FLASH_OFF_TIME);
    }

    private void lightsOff() {
        appendages.setBlinkinPattern(BlinkinPatterns.OFF);
    }
}
