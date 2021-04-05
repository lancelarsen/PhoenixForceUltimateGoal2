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

@TeleOp(group = "auto")
public class RedInsidePowerShot extends AbstractAuto {
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
        drive.line(FieldPositions.S3W);
        appendages.wobbleGoalGrab();

        //--- Shoot power shots
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
        drive.line(FieldPositions.P4);
        sleep(500);
        for (int i = 0; i < 3; i++) {
            appendages.shootRings(1);
            sleep(2000);

            if (i == 2) break;

            drive.turnLeft(FieldPositions.P4A[i]);
            sleep(2000);
        }
        appendages.shooterOff();

        //--- Pick up rings near power shots
        appendages.ringIntakeStart();
        drive.curve(FieldPositions.C2A);
        drive.setSpeed(MecanumAutonomous.Speed.VERY_SLOW);
        drive.curve(FieldPositions.C2B);
        drive.setSpeed(MecanumAutonomous.Speed.FAST);
        appendages.ringIntakeStop();

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
