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
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Disabled
@Autonomous(group = "auto")
public class R_OUT_PS_WG_L2 extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.OUTSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        //--- Grab wobble goal
        drive.line(FieldPositions.S4W);
        appendages.wobbleGoalGrab();

        //--- Shoot power shots
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
        drive.line(FieldPositions.P6);
        for (int i = 0; i < 3; i++) {
            appendages.shootRings(1);
            sleep(500);

            if (i == 2) break;
            drive.turnLeft(FieldPositions.P6A[i]);
            sleep(500);
        }
        appendages.shooterOff();

        //--- Drop wobble goal
        switch (targetZone) {
            case ZONE_A:
                drive.line(FieldPositions.W4O); break;
            case ZONE_B:
                drive.line(FieldPositions.W5O); break;
            case ZONE_C:
                drive.line(FieldPositions.W6O); break;
        }
        appendages.wobbleGoalDrop();

        //--- Park on line
        drive.line(FieldPositions.X3R);
        //TODO: wait until 5 seconds left
        drive.line(FieldPositions.L2R);

//        //--- Return to start
//        sleep(5000);
//        drive.setSpeed(MecanumAutonomous.Speed.MEDIUM);
//        drive.line(FieldPositions.S4);
//        appendages.wobbleGoalDrop();
    }
}
