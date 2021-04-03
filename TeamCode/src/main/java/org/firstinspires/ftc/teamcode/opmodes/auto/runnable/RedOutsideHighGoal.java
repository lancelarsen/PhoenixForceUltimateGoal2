package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@TeleOp(name = "AAA Red->Outside HG", group = "drive")
public class RedOutsideHighGoal extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.OUTSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        switch (targetZone) {
            case ZONE_A:
                // Zone A
            case ZONE_B:
                // Zone B
            case ZONE_C:
                // Zone C
        }

        //--- Grab wobble goal
        drive.line(FieldPositions.S4W);
        appendages.wobbleGoalGrab();

        //--- Shoot power shots
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
        drive.curve(FieldPositions.T5);
        for (int i = 0; i < 3; i++) {
            sleep(500);
            appendages.shootRings(1);
        }
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.OFF);

        //--- Pick up rings near power shots
//        appendages.ringIntakeStart();
//        drive.curve(FieldPositions.C2A);
//        drive.setSpeed(MecanumAutonomous.Speed.VERY_SLOW);
//        drive.curve(FieldPositions.C2B);
//        drive.setSpeed(MecanumAutonomous.Speed.FAST);
//        appendages.ringIntakeStop();

        //--- Drop wobble goal
//        drive.line(FieldPositions.W5S);
//        appendages.wobbleGoalDrop();

        //--- Park on line
//        drive.line(FieldPositions.X3R);
//        drive.line(FieldPositions.L2R);

        //--- Return to start
        sleep(5000);
        drive.setSpeed(MecanumAutonomous.Speed.MEDIUM);
        drive.line(FieldPositions.S4);
        appendages.wobbleGoalDrop();
    }
}
