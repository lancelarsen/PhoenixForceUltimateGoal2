package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FullAuto;

@Autonomous(group = "drive")
public class RedFullAutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        FullAuto.init(this, AutoUtils.Alliance.RED, AutoUtils.StartingPosition.RED_LEFT);

        waitForStart();
        if (isStopRequested()) return;

        FullAuto.run();


        /*waitForStart();
        if (isStopRequested()) return;
        BotMecanumDrive drive = new BotMecanumDrive(hardwareMap);

        //Pose2d initialPosition = new Pose2d(-62, -50, Math.toRadians(0));
        //Pose2d initialPosition = new Pose2d(20, 0, 0);
        //drive.setPoseEstimate(initialPosition);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .back(20)
                //.strafeLeft(20)
                //.lineToConstantHeading(new Vector2d(-20, 0))
                //.splineToConstantHeading(new Vector2d(5, -40), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(5, -60, Math.toRadians(0)), Math.toRadians(180))
                .build();

        drive.followTrajectory(trajectory);
        drive.followTrajectory(drive.trajectoryBuilder(trajectory.end())
                //.back(20)
                .strafeLeft(20)
                //.lineToConstantHeading(new Vector2d(-20, 0))
                //.splineToConstantHeading(new Vector2d(5, -40), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(5, -60, Math.toRadians(0)), Math.toRadians(180))
                .build());*/
    }
}
