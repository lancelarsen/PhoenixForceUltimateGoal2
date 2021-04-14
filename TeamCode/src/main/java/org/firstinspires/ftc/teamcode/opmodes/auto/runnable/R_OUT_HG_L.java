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
public class R_OUT_HG_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.OUTSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        //--- Grab wobble goal
        drive.line(FieldPositions.S4W);
        appendages.wobbleGoalGrab();

        //--- Shoot top goal
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
        drive.curve(FieldPositions.T5);
        appendages.shootRings();
        appendages.shooterOff();

        //--- Drop wobble goal
        switch (targetZone) {
            case ZONE_A:
                drive.line(FieldPositions.X4);
                sleep(10000);
                drive.line(FieldPositions.W4O);
                break;
            case ZONE_B:
                drive.line(FieldPositions.W5O); break;
            case ZONE_C:
                drive.line(FieldPositions.W6O); break;
        }
        appendages.wobbleGoalDrop();

        //--- Park on line
//        switch (targetZone) {
//            case ZONE_A:
//                drive.line(FieldPositions.L2R); break;
//            case ZONE_B:
//            case ZONE_C:
//                drive.line(FieldPositions.L2); break;
//        }

        //--- Return to start
        sleep(5000);
        drive.setSpeed(MecanumAutonomous.Speed.MEDIUM);
        drive.line(FieldPositions.RSO);
        appendages.wobbleGoalDrop();
    }
}
