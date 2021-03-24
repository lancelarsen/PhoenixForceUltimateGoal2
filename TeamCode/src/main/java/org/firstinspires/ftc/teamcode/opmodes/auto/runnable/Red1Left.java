package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FullAuto;
import org.firstinspires.ftc.teamcode.vision.RingVision;

@Autonomous(group = "drive")
public class Red1Left extends LinearOpMode {

    private static LinearOpMode opMode;
    private static AutoUtils.Alliance alliance;
    private static AutoUtils.StartingPosition startingPosition;

    private static AppendagesAutonomous appendages;
    private static RingVision ringVision;

    public static void init(LinearOpMode _opMode, AutoUtils.Alliance _alliance, AutoUtils.StartingPosition _startingPosition) {
        opMode = _opMode;
        alliance = _alliance;
        startingPosition = _startingPosition;

        appendages = new AppendagesAutonomous(opMode);
        ringVision = new RingVision(opMode.hardwareMap);

        appendages.openGoalGrabber(true);
        //appendages.setShooterTilterAngle(BotAppendages.ShooterAngle.SHOOTING);

        ringVision.init(startingPosition);
        ringVision.setViewportPaused(false);

        appendages.setBlinkinPattern(alliance == AutoUtils.Alliance.RED ? RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED : RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
    }


    @Override
    public void runOpMode() {
        //FullAuto.init(this, AutoUtils.Alliance.RED, AutoUtils.StartingPosition.RED_LEFT);



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
