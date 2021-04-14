package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.sleep;

@Autonomous(group = "auto")
public class R_OUT_L extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.RED, AutoUtils.StartingPosition.OUTSIDE);

        //--- Detect number of rings
        //RingVision.TargetZone targetZone = ringVision.getTargetZone();

        //--- Grab wobble goal
//        drive.line(FieldPositions.S4W);
//        appendages.wobbleGoalGrab();

        //--- Wait for 20 seconds
//        sleep(20000);
//
//        //--- park on line, drop wobble goal
//        drive.line(FieldPositions.L3);
//        appendages.wobbleGoalDrop();

        //--- temporary
//        sleep(4000);
//        drive.line(FieldPositions.S4);


        //--- Shoot top goal
        drive.setCurrentPosition(FieldPositions.RSA);
        shootPowershotsFromSide(FieldPositions.RP1, FieldPositions.RP1A);

        appendages.shooterOff();

        //--- Wait for 20 seconds
        sleep(5000);
    }

    private void shootPowershotsFromSide(Pose2d shootingPose, double turnAngles[]) {
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
        drive.line(shootingPose);
        for (int i = 0; i < 3; i++) {
            appendages.shootRings(1);
            if (i == 2) break;
            drive.turnLeft(turnAngles[i]);
        }
    }
}
