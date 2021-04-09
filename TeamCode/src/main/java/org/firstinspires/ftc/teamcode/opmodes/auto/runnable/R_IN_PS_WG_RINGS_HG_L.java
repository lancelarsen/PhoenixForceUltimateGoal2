package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Disabled
@Autonomous(group = "auto")
public class R_IN_PS_WG_RINGS_HG_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.INSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        //--- Grab wobble goal
        drive.line(FieldPositions.S3W);
        appendages.wobbleGoalGrab();

        //--- Shoot power shots
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
        drive.line(FieldPositions.P4);
        for (int i = 0; i < 3; i++) {
            appendages.shootRings(1);
            sleep(500);

            if (i == 2) break;
            drive.turnLeft(FieldPositions.P4A[i]);
            sleep(500);
        }
        appendages.shooterOff();

        //--- Drop wobble goal
        switch (targetZone) {
            case ZONE_A:
                drive.line(FieldPositions.X4);
                sleep(10000);
                drive.line(FieldPositions.W4I);
                break;
            case ZONE_B:
                drive.line(FieldPositions.W5I); break;
            case ZONE_C:
                drive.line(FieldPositions.W6I); break;
        }
        appendages.wobbleGoalDrop();

        //--- Go get rings and shoot!
        switch (targetZone) {
            case ZONE_A:
                break;
            case ZONE_B:
                drive.line(FieldPositions.X4);
                appendages.ringIntakeStart();
                appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
                drive.line(FieldPositions.R4A);
                drive.setSpeed(MecanumAutonomous.Speed.VERY_SLOW);
                drive.line(FieldPositions.R4B);
                drive.setSpeed(MecanumAutonomous.Speed.FAST);
                drive.line(FieldPositions.T4);
                drive.turnLeft(8);
                appendages.shootRings();
                appendages.shooterOff();
                break;
            case ZONE_C:
                //TODO: Count rings to stop taking in too many
                appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
                drive.line(FieldPositions.R4B);
                drive.turnLeft(15);
                drive.setSpeed(MecanumAutonomous.Speed.VERY_SLOW);
                appendages.ringIntakeStart();
                drive.line(FieldPositions.R4C);
                appendages.ringIntakeStart();
                drive.setSpeed(MecanumAutonomous.Speed.FAST);
                drive.curve(FieldPositions.T4);
                drive.turnRight(9);
                appendages.shootRings();
                appendages.shooterOff();
                appendages.ringIntakeStop();
                break;
        }

        //--- Park on line
        switch (targetZone) {
            case ZONE_A:
                drive.line(FieldPositions.L2R); break;
            case ZONE_B:
            case ZONE_C:
                drive.line(FieldPositions.L2); break;
        }
    }
}