package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.BotMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.RingVision;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.Alliance;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.StartingPosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.sleep;

public class FullAuto {
    private enum TargetZone {
        ZONE_A,
        ZONE_B,
        ZONE_C,
    }

    private static LinearOpMode opMode;
    private static Alliance alliance;
    private static StartingPosition startingPosition;

    private static AppendagesAutonomous appendages;
    private static RingVision ringVision;

    public static void init(LinearOpMode _opMode, Alliance _alliance, StartingPosition _startingPosition) {
        opMode = _opMode;
        alliance = _alliance;
        startingPosition = _startingPosition;

        appendages = new AppendagesAutonomous(opMode);
        ringVision = new RingVision(opMode.hardwareMap);

        appendages.openGoalGrabber(true);
        appendages.setShooterTilterAngle(BotAppendages.ShooterAngle.SHOOTING);

        ringVision.init(startingPosition);
        ringVision.setViewportPaused(false);

        appendages.setBlinkinPattern(alliance == Alliance.RED ? RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED : RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
    }

    public static void run() {

        RingVision.RingCount ringCount = ringVision.getRingCount();
        ringVision.setViewportPaused(true);

        /*appendages.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        RingVision.RingCount ringCount = ringVision.getRingCount();
        ringVision.setViewportPaused(true);

        TargetZone targetZone = TargetZone.ZONE_C;
        switch (ringCount) {
            case ZERO:
                targetZone = TargetZone.ZONE_A;
                break;
            case ONE:
                targetZone = TargetZone.ZONE_B;
                break;
        }*/

        //opMode.telemetry.addData("Target zone", targetZone);
        //opMode.telemetry.update();

        //Pose2d initialPosition = new Pose2d(-62, -50, Math.toRadians(0));
        //Pose2d initialPosition = new Pose2d(20, 0, 0);
        //drive.setPoseEstimate(initialPosition);

        //appendages.openGoalGrabber(false);
        //sleep(1000);
        BotMecanumDrive drive = new BotMecanumDrive(opMode.hardwareMap);

        //appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.MIDDLE);
//        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
//                .back(20)
//                //.strafeLeft(20)
//                //.lineToConstantHeading(new Vector2d(-20, 0))
//                //.splineToConstantHeading(new Vector2d(5, -40), Math.toRadians(0))
//                //.splineToLinearHeading(new Pose2d(5, -60, Math.toRadians(0)), Math.toRadians(180))
//                .build();

//        drive.followTrajectory(trajectory);
//        drive.followTrajectory(drive.trajectoryBuilder(trajectory.end())
//                //.back(20)
//                .strafeLeft(20)
//                //.lineToConstantHeading(new Vector2d(-20, 0))
//                //.splineToConstantHeading(new Vector2d(5, -40), Math.toRadians(0))
//                //.splineToLinearHeading(new Pose2d(5, -60, Math.toRadians(0)), Math.toRadians(180))
//                .build());
//        sleep(5000);

        /*Pose2d endPosition = driveToZone(drive, initialPosition, targetZone, 7);

        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.DOWN);
        sleep(1000);
        appendages.openGoalGrabber(true);
        sleep(1000);

        appendages.enableShooterWheel(true);

        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.UP);

        Trajectory driveToShootingLocation = drive.trajectoryBuilder(endPosition, false)
                .splineToLinearHeading(new Pose2d(-15, alliance == Alliance.RED ? -31 : 31, Math.toRadians(0)), Math.toRadians(180))
                .build();

        drive.followTrajectory(driveToShootingLocation);
        endPosition = driveToShootingLocation.end();

        sleep(1000);

        for(int i = 0; i < 3; i++) {
            appendages.extendShooterArm(true);
            sleep(350);
            appendages.extendShooterArm(false);
            sleep(700);
        }

        appendages.enableShooterWheel(false);

        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.DOWN);
        appendages.openGoalGrabber(true);

        Trajectory driveToSecondGoal = drive.trajectoryBuilder(endPosition, true)
                .splineToConstantHeading(new Vector2d(-32, alliance == Alliance.RED ? -22 : 22), Math.toRadians(0))
                .build();

        drive.followTrajectory(driveToSecondGoal);
        endPosition = driveToSecondGoal.end();

        appendages.openGoalGrabber(false);
        sleep(1000);
        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.MIDDLE);

        endPosition = driveToZone(drive, endPosition, targetZone, -7);

        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.DOWN);
        sleep(1000);
        appendages.openGoalGrabber(true);
        sleep(1000);
        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.MIDDLE);
        appendages.openGoalGrabber(false);

        TrajectoryBuilder driveToLineBuilder = drive.trajectoryBuilder(endPosition, false);
        if (targetZone == TargetZone.ZONE_A) {
            driveToLineBuilder.splineToConstantHeading(new Vector2d(-8, alliance == Alliance.RED ? -50 : 50), Math.toRadians(90));
        }
        driveToLineBuilder.splineToConstantHeading(new Vector2d(12, alliance == Alliance.RED ? -40 : 40), Math.toRadians(0));

        drive.followTrajectory(driveToLineBuilder.build());*/
    }

    Pose2d driveToZone(BotMecanumDrive drive, Pose2d startingPosition, TargetZone targetZone, double xOffset) {
        Trajectory driveToA = drive.trajectoryBuilder(startingPosition, false)
                .splineTo(new Vector2d(-2 + xOffset, alliance == Alliance.RED ? -60 : 60), Math.toRadians(0))
                .build();

        Trajectory driveToB = drive.trajectoryBuilder(startingPosition, false)
                .splineTo(new Vector2d(-15, alliance == Alliance.RED ? -50 : 50), Math.toRadians(0))
                .splineTo(new Vector2d(20 + xOffset, alliance == Alliance.RED ? -37 : 37), Math.toRadians(0))
                .build();

        Trajectory driveToC = drive.trajectoryBuilder(startingPosition, false)
                .splineTo(new Vector2d(-15, alliance == Alliance.RED ? -50 : 50), Math.toRadians(0))
                .splineTo(new Vector2d(45 + xOffset, (xOffset == -7 ? -65 : -60) * (alliance == Alliance.RED ? 1 : -1)), Math.toRadians(0))
                .build();


        switch(targetZone) {
            case ZONE_A:
                drive.followTrajectory(driveToA);
                return driveToA.end();
            case ZONE_B:
                drive.followTrajectory(driveToB);
                return driveToB.end();
            case ZONE_C:
                drive.followTrajectory(driveToC);
                return driveToC.end();
            default:
                return new Pose2d();
        }
    }
}
