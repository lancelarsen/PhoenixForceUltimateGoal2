package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Autonomous(group = "auto")
public class R_OUT_HG_WG_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.OUTSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        //--- Grab wobble goal
        drive.line(FieldPositions.S4W);
        appendages.wobbleGoalGrab();

        //--- delays
        switch (targetZone) {
            case ZONE_A:
                break;
            case ZONE_B:
            case ZONE_C:
                sleep(10000); break;
        }

        //--- Shoot top goal
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
        drive.curve(FieldPositions.T5);
        appendages.shootRings();
        appendages.shooterOff();

        //--- Drop wobble goal
        switch (targetZone) {
            case ZONE_A:
                drive.line(FieldPositions.W4OB); break;
            case ZONE_B:
                drive.line(FieldPositions.W5OB); break;
            case ZONE_C:
                drive.line(FieldPositions.W6OB); break;
        }
        appendages.wobbleGoalDrop();

        //--- Park on line
        switch (targetZone) {
            case ZONE_A:
                drive.line(FieldPositions.X6A);
                sleep(10000);
                break;
            case ZONE_B:
            case ZONE_C:
                break;
        }
        drive.line(FieldPositions.X6B);
        appendages.setReachArmPosition(BotAppendages.ReachArmPosition.EXTENDED);
    }
}
