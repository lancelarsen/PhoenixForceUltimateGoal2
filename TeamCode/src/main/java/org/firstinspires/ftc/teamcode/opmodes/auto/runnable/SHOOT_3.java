package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.opmodes.auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Autonomous(group = "auto")
public class SHOOT_3 extends AbstractAuto {
    public void runOpMode() {
        initAuto(AutoUtils.Alliance.BLUE, AutoUtils.StartingPosition.INSIDE);

        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
        sleep(2000);
        appendages.shootRings(3);
        appendages.shooterOff();
    }
}
