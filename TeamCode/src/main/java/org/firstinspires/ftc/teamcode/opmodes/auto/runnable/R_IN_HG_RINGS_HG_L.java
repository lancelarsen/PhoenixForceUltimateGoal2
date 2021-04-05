package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@TeleOp(group = "auto")
public class R_IN_HG_RINGS_HG_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.INSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        //TODO

//        //--- Grab wobble goal
//        drive.line(FieldPositions.S3W);
//        appendages.wobbleGoalGrab();
//
//        //--- Shoot top goal
//        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
//        drive.curve(FieldPositions.T7);
//        appendages.shootRings();
//        appendages.shooterOff();
//
//        //--- Move to center point in back field
//        drive.curve(FieldPositions.C2B);
//
//        //--- Drop wobble goal
//        //drive.turnLeft(90);
//        drive.line(FieldPositions.W5S);
//        appendages.wobbleGoalDrop();
//
//        //--- Park on line
//        drive.line(FieldPositions.X3R);
//        drive.line(FieldPositions.L2R);
//
//        //--- Return to start
//        sleep(5000);
//        drive.setSpeed(MecanumAutonomous.Speed.MEDIUM);
//        drive.line(FieldPositions.S3);
    }
}
