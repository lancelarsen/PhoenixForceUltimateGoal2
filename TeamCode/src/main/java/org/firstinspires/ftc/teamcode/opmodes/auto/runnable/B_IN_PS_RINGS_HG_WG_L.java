package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Autonomous(group = "auto")
public class B_IN_PS_RINGS_HG_WG_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.BLUE, AutoUtils.StartingPosition.INSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        appendages.ringIntakeStart();

        //--- Grab wobble goal
        drive.line(FieldPositions.S2W);
        appendages.wobbleGoalGrab();

        //--- Shoot power shots
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
        drive.line(FieldPositions.P2);
        for (int i = 0; i < 3; i++) {
            appendages.shootRings(1);
            sleep(500);

            if (i == 2) break;
            drive.turnLeft(FieldPositions.P2A[i]);
            sleep(500);
        }
        appendages.shooterOff();

        //--- Go get rings and shoot!
        switch (targetZone) {
            case ZONE_A:
                break;
            case ZONE_B:
                drive.turnRight(15);
                drive.line(FieldPositions.R2A);
                appendages.ringIntakeStart();
                drive.setSpeed(MecanumAutonomous.Speed.VERY_SLOW);
                drive.line(FieldPositions.R2B);
                appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
                drive.setSpeed(MecanumAutonomous.Speed.FAST);
                drive.turnLeft(15);
                drive.curve(FieldPositions.T1);
                appendages.shootRings();
                appendages.shooterOff();
                appendages.ringIntakeStop();
                break;
            case ZONE_C:
                //TODO: Count rings to stop taking in too many
                drive.turnRight(15);
                drive.line(FieldPositions.R2A);
                drive.line(FieldPositions.R2B);
                drive.setSpeed(MecanumAutonomous.Speed.VERY_SLOW);
                appendages.ringIntakeStart();
                drive.line(FieldPositions.R2C);
                appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
                drive.setSpeed(MecanumAutonomous.Speed.FAST);
                drive.curve(FieldPositions.T1);
                appendages.shootRings();
                appendages.shooterOff();
                appendages.ringIntakeStop();
                break;
        }

        //--- Drop wobble goal
        switch (targetZone) {
            case ZONE_A:
                drive.line(FieldPositions.X2);
                sleep(10000);
                drive.line(FieldPositions.W1I);
                break;
            case ZONE_B:
                drive.line(FieldPositions.W2I);
                break;
            case ZONE_C:
                drive.line(FieldPositions.W3I); break;
        }
        appendages.wobbleGoalDrop();

        //--- Park on line
        switch (targetZone) {
            case ZONE_A:
            case ZONE_B:
            case ZONE_C:
                drive.line(FieldPositions.L1); break;
        }
    }
}