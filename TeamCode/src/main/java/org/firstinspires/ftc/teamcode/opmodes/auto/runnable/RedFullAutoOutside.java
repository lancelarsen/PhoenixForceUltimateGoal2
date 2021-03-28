package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FullAuto;

@Autonomous(group = "drive")
public class RedFullAutoOutside extends LinearOpMode {
    @Override
    public void runOpMode() {
        FullAuto auto = new FullAuto(this, AutoUtils.Alliance.RED, AutoUtils.StartingPosition.OUTSIDE);
        auto.init();

        waitForStart();
        if (isStopRequested()) return;

        auto.run();
    }
}
