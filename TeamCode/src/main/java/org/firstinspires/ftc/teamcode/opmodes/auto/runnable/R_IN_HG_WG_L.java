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
public class R_IN_HG_WG_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.INSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        appendages.ringIntakeStart();

        //--- Grab wobble goal
        drive.line(FieldPositions.S3W);
        appendages.wobbleGoalGrab();

        //--- Shoot top goal
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
        drive.curve(FieldPositions.T4);
        appendages.shootRings();
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