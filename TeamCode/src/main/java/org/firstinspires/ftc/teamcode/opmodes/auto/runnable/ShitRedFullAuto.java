//package org.firstinspires.ftc.teamcode.opmodes.auto.runnable;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
//import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
//import org.firstinspires.ftc.teamcode.drive.BotMecanumDrive;
//import org.firstinspires.ftc.teamcode.vision.RingVision;
//
//import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.Alliance;
//import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.sleep;
//
//@Autonomous(group = "drive")
//public class ShitRedFullAuto extends LinearOpMode {
//    private enum TargetZone {
//        ZONE_A,
//        ZONE_B,
//        ZONE_C,
//    }
//
//    private Alliance alliance = Alliance.RED;
//
//    private BotMecanumDrive drive;
//    private AppendagesAutonomous appendages;
//    private RingVision ringVision;
//
//    private Pose2d initialPosition;
//
//    @Override
//    public void runOpMode() {
//        drive = new BotMecanumDrive(hardwareMap);
//        appendages = new AppendagesAutonomous(this);
//        ringVision = new RingVision(hardwareMap);
//
//        initialPosition = new Pose2d(-62, alliance == Alliance.RED ? -50 : 50, Math.toRadians(180));
//        drive.setPoseEstimate(initialPosition);
//
//        appendages.openGoalGrabber(true);
//        appendages.setShooterTilterAngle(BotAppendages.ShooterAngle.SHOOTING);
//
//        ringVision.init();
//        ringVision.setViewportPaused(false);
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        RingVision.RingCount ringCount = ringVision.getRingCount();
//        ringVision.setViewportPaused(true);
//
//        TargetZone targetZone = TargetZone.ZONE_C;
//        switch (ringCount) {
//            case ZERO:
//                targetZone = TargetZone.ZONE_A;
//                break;
//            case ONE:
//                targetZone = TargetZone.ZONE_B;
//                break;
//        }
//
//        telemetry.addData("Target zone", targetZone);
//        telemetry.update();
//
//        appendages.openGoalGrabber(false);
//        sleep(1000);
//        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.MIDDLE);
//
//        Pose2d endPosition = driveToZone(drive, initialPosition, targetZone, 7);
//
//        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.DOWN);
//        sleep(1000);
//        appendages.openGoalGrabber(true);
//        sleep(1000);
//
//        appendages.enableShooterWheel(true);
//
//        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.UP);
//
//        Trajectory driveToShootingLocation = drive.trajectoryBuilder(endPosition, false)
//                .splineToLinearHeading(new Pose2d(-15, alliance == Alliance.RED ? -31 : 31, Math.toRadians(0)), Math.toRadians(180))
//                .build();
//
//        drive.followTrajectory(driveToShootingLocation);
//        endPosition = driveToShootingLocation.end();
//
//        sleep(1000);
//
//        for(int i = 0; i < 3; i++) {
//            appendages.extendShooterArm(true);
//            sleep(350);
//            appendages.extendShooterArm(false);
//            sleep(700);
//        }
//
//        appendages.enableShooterWheel(false);
//
//        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.DOWN);
//        appendages.openGoalGrabber(true);
//
//        Trajectory driveToSecondGoal = drive.trajectoryBuilder(endPosition, true)
//                .splineToConstantHeading(new Vector2d(-32, alliance == Alliance.RED ? -22 : 22), Math.toRadians(0))
//                .build();
//
//        drive.followTrajectory(driveToSecondGoal);
//        endPosition = driveToSecondGoal.end();
//
//        appendages.openGoalGrabber(false);
//        sleep(1000);
//        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.MIDDLE);
//
//        endPosition = driveToZone(drive, endPosition, targetZone, -7);
//
//        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.DOWN);
//        sleep(1000);
//        appendages.openGoalGrabber(true);
//        sleep(1000);
//        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.MIDDLE);
//        appendages.openGoalGrabber(false);
//
//        TrajectoryBuilder driveToLineBuilder = drive.trajectoryBuilder(endPosition, false);
//        if (targetZone == TargetZone.ZONE_A) {
//            driveToLineBuilder.splineToConstantHeading(new Vector2d(-8, alliance == Alliance.RED ? -50 : 50), Math.toRadians(90));
//        }
//        driveToLineBuilder.splineToConstantHeading(new Vector2d(12, alliance == Alliance.RED ? -40 : 40), Math.toRadians(0));
//
//        drive.followTrajectory(driveToLineBuilder.build());
//    }
//
//    Pose2d driveToZone(BotMecanumDrive drive, Pose2d startingPosition, TargetZone targetZone, double xOffset) {
//        Trajectory driveToA = drive.trajectoryBuilder(startingPosition, true)
//                .splineTo(new Vector2d(-2 + xOffset, alliance == Alliance.RED ? -60 : 60), Math.toRadians(0))
//                .build();
//
//        Trajectory driveToB = drive.trajectoryBuilder(startingPosition, true)
//                .splineTo(new Vector2d(-15, alliance == Alliance.RED ? -50 : 50), Math.toRadians(0))
//                .splineTo(new Vector2d(20 + xOffset, alliance == Alliance.RED ? -37 : 37), Math.toRadians(0))
//                .build();
//
//        Trajectory driveToC = drive.trajectoryBuilder(startingPosition, true)
//                .splineTo(new Vector2d(-15, alliance == Alliance.RED ? -50 : 50), Math.toRadians(0))
//                .splineTo(new Vector2d(45 + xOffset, (xOffset == -7 ? -65 : -60) * (alliance == Alliance.RED ? 1 : -1)), Math.toRadians(0))
//                .build();
//
//
//        switch(targetZone) {
//            case ZONE_A:
//                drive.followTrajectory(driveToA);
//                return driveToA.end();
//            case ZONE_B:
//                drive.followTrajectory(driveToB);
//                return driveToB.end();
//            case ZONE_C:
//                drive.followTrajectory(driveToC);
//                return driveToC.end();
//            default:
//                return new Pose2d();
//        }
//    }
//}