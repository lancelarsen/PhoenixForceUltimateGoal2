package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;

public class NotShitDemo extends LinearOpMode {
    public void runOpMode() {
        MecanumAutonomous drive = new MecanumAutonomous(this);
        AppendagesAutonomous appendages = new AppendagesAutonomous(this);

        waitForStart();
        if (isStopRequested()) return;

        //drive.setCurrentPosition(FieldPositions.startingPose);

//        drive.builder(MecanumAuto.Speed.FAST)
//            .line(FieldPositions.testPoint1)
//            .spline(FieldPositions.testPoint2, 180)
//            .splineConstantHeading(FieldPositions.testPoint1, 180)
//            .follow();

        appendages.openGoalGrabber(true);
    }
}
