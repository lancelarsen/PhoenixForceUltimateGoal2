package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.vision.RingVision;

abstract public class AbstractAuto extends LinearOpMode {
    public MecanumAutonomous drive;
    public AppendagesAutonomous appendages;
    public RingVision ringVision;

    public void initAuto(AutoUtils.Alliance alliance, AutoUtils.StartingPosition startingPosition) {
        drive = new MecanumAutonomous(this);
        appendages = new AppendagesAutonomous(this);
        ringVision = new RingVision(hardwareMap);

        ringVision.init(alliance, startingPosition);

        if (alliance == AutoUtils.Alliance.BLUE) {
            if (startingPosition == AutoUtils.StartingPosition.OUTSIDE) {
                drive.setCurrentPosition(FieldPositions.S1);
            } else {
                drive.setCurrentPosition(FieldPositions.S2);
            }
            appendages.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        } else {
            if (startingPosition == AutoUtils.StartingPosition.INSIDE) {
                drive.setCurrentPosition(FieldPositions.S3);
            } else {
                drive.setCurrentPosition(FieldPositions.S4);
            }
            appendages.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        }

        drive.setSpeed(MecanumAutonomous.Speed.FAST);

        waitForStart();
        if (isStopRequested()) return;

        ringVision.setViewportPaused(true);
    }
}
