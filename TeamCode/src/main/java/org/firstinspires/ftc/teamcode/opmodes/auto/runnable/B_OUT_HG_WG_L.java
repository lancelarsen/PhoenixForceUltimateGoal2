package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Autonomous(group = "auto")
public class B_OUT_HG_WG_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.BLUE, AutoUtils.StartingPosition.OUTSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        appendages.ringIntakeStart();

        //--- Grab wobble goal
        drive.line(FieldPositions.S1W);
        appendages.wobbleGoalGrab();

     //   //--- delays
        switch (targetZone) {
            case ZONE_A:
                break;
            case ZONE_B:
            case ZONE_C:
                // sleep(10000);
                break;
        }

        //--- Shoot top goal
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
        drive.curve(FieldPositions.T2);
        appendages.shootRings();
        appendages.shooterOff();

        //--- Drop wobble goal
        switch (targetZone) {
           case ZONE_A:
                drive.line(FieldPositions.W1); break;
            case ZONE_B:
                drive.line(FieldPositions.W2); break;
            case ZONE_C:
                drive.line(FieldPositions.W3); break;
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
                drive.line(FieldPositions.BL1);
                break;
        }
 //       drive.line(FieldPositions.BL1);
        appendages.setReachArmPosition(BotAppendages.ReachArmPosition.EXTENDED);
    }
}
