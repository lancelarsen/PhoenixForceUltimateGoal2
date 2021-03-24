package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FullAuto;

@Autonomous(group = "drive")
public class RedFullAutoRight extends LinearOpMode {
    @Override
    public void runOpMode() {
        FullAuto.init(this, AutoUtils.Alliance.RED, AutoUtils.StartingPosition.RED_RIGHT);

        waitForStart();
        if (isStopRequested()) return;

        FullAuto.run();
    }
}
