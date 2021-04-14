package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Disabled
@Autonomous(group = "auto")
public class R_IN_PS_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.INSIDE);

        //--- Detect number of rings
        RingVision.TargetZone targetZone = ringVision.getTargetZone();

        //--- Grab wobble goal
        drive.line(FieldPositions.RSI_W);
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
                drive.line(FieldPositions.RX2);
                sleep(10000);
                drive.line(FieldPositions.RI_WA);
                break;
            case ZONE_B:
                drive.line(FieldPositions.RI_WB); break;
            case ZONE_C:
                drive.line(FieldPositions.RI_WC); break;
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
