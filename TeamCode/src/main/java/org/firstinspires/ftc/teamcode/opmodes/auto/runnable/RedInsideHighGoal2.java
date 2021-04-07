package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;

@Disabled
@Autonomous(group = "auto")
public class RedInsideHighGoal2 extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.OUTSIDE);

        //--- Grab wobble goal
        drive.line(FieldPositions.S3W);
        appendages.wobbleGoalGrab();

        //--- Shoot top goal
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
        drive.curve(FieldPositions.T4);
        appendages.shootRings();
        appendages.shooterOff();

        //--- Move to center point in back field
        drive.curve(FieldPositions.C2B);

        //--- Drop wobble goal
        //drive.turnLeft(90);
        drive.line(FieldPositions.W5S);
        appendages.wobbleGoalDrop();

        //--- Park on line
        drive.line(FieldPositions.X3R);
        drive.line(FieldPositions.L2R);

        //--- Return to start
        sleep(5000);
        drive.setSpeed(MecanumAutonomous.Speed.MEDIUM);
        drive.line(FieldPositions.S3);
    }
}
