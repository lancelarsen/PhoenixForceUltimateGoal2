package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Autonomous(group = "auto")
public class R_IN_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.INSIDE);

        //--- Detect number of rings
        //RingVision.TargetZone targetZone = ringVision.getTargetZone();

        //--- Grab wobble goal
        //drive.line(FieldPositions.S3W);
        //appendages.wobbleGoalGrab();

        drive.setCurrentPosition(FieldPositions.RSA);

        //--- Shoot top goal
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
        drive.line(FieldPositions.RSB);
        sleep(1000);
        //rive.curve(FieldPositions.T5);
        appendages.shootRings();
        appendages.shooterOff();

        drive.line(FieldPositions.RSA);

        //--- Wait for 20 seconds
        sleep(5000);

        //--- park on line, drop wobble goal
        //drive.line(FieldPositions.L2);
        //appendages.wobbleGoalDrop();
    }
}
