package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Autonomous(group = "auto")
public class _B_OUT_HG_WG_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.BLUE, AutoUtils.StartingPosition.OUTSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        appendages.ringIntakeStart();

        //--- Grab wobble goal
        drive.line(FieldPositions.BSO_W);
        appendages.wobbleGoalGrab();

        //--- delays
        switch (targetZone) {
            case ZONE_A:
                break;
            case ZONE_B:
            case ZONE_C:
                sleep(10000);
                break;
        }

        //--- Shoot top goal
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
        drive.curve(FieldPositions.BTO2);
        appendages.shootRings();
        appendages.shooterOff();

        //--- Drop wobble goal
        switch (targetZone) {
           case ZONE_A:
                drive.line(FieldPositions.BO_WA); break;
            case ZONE_B:
                drive.line(FieldPositions.BO_WB); break;
            case ZONE_C:
                drive.line(FieldPositions.BO_WC); break;
        }
        appendages.wobbleGoalDrop();

        //--- Park on line
       switch (targetZone) {
            case ZONE_A:
                drive.line(FieldPositions.BX6);
                sleep(10000);
                break;
            case ZONE_B:
            case ZONE_C:
                drive.line(FieldPositions.BLO);
                break;
        }
        //TODO: What should we do about parking for A?
 //       drive.line(FieldPositions.BL1);
        appendages.setReachArmPosition(BotAppendages.ReachArmPosition.EXTENDED);
    }
}
