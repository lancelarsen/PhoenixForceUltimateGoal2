/*package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.MecanumAuto;
import org.firstinspires.ftc.teamcode.vision.RingVision;

import static org.firstinspires.ftc.teamcode.drive.MecanumAuto.Speed.FAST;
import static org.firstinspires.ftc.teamcode.drive.MecanumAuto.Speed.MEDIUM;
import static org.firstinspires.ftc.teamcode.drive.MecanumAuto.Speed.SLOW;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.Alliance;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.StartingPosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.sleep;

public class FullAuto {
    private LinearOpMode opMode;
    private StartingPosition startingPosition;
    private Alliance alliance;

    private MecanumAuto drive;
    private AppendagesAutonomous appendages;
    private RingVision ringVision;

    public FullAuto(LinearOpMode opMode, Alliance alliance, StartingPosition startingPosition) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.startingPosition = startingPosition;
    }

    public void init() {
        drive = new MecanumAuto(opMode);
        appendages = new AppendagesAutonomous(opMode);
        ringVision = new RingVision(opMode.hardwareMap);

        drive.setCurrentPosition(FieldPositions.RED_OUTSIDE_START);

        appendages.openGoalGrabber(true);
        appendages.setShooterTilterAngle(BotAppendages.ShooterAngle.SHOOTING);

        ringVision.init(alliance, startingPosition);
        ringVision.setViewportPaused(false);
    }

    public void run() {
        RingVision.TargetZone targetZone = ringVision.getTargetZone();
        ringVision.setViewportPaused(true);

        MecanumAuto.Builder builder;
        Pose2d startingPose;
        if (alliance == Alliance.BLUE) {
            if (startingPosition == StartingPosition.OUTSIDE) {
                startingPose = FieldPositions.S4;
            } else {
                startingPose = FieldPositions.S3;
            }
        } else {
            if (startingPosition == StartingPosition.OUTSIDE) {
                startingPose = FieldPositions.S2;
            } else {
                startingPose = FieldPositions.S1;
            }
        }

        opMode.telemetry.addData("Target zone", targetZone);
        opMode.telemetry.update();

        Pose2d grabFirstGoalPose = startingPose.plus(new Pose2d(2, 0));

        builder = drive.builder(FAST);
        if (startingPose == FieldPositions.S1) {
            builder.spline(FieldPositions.S1W, 0);
        } else if (startingPose == FieldPositions.S2) {
            builder.spline(FieldPositions.S2W, 0);
        } else if (startingPose == FieldPositions.S3) {
            builder.spline(FieldPositions.S3W, 0);
        } else if (startingPose == FieldPositions.S4) {
            builder.spline(FieldPositions.S4W, 0);
        }
        builder.follow();

        appendages.openGoalGrabber(false);
        sleep(1000);
        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.MIDDLE);

        driveToZone(targetZone, 7);

        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.DOWN);
        sleep(1000);
        appendages.openGoalGrabber(true);
        sleep(1000);

        appendages.enableShooterWheel(true);

        drive.builder(FAST)
            .spline(FieldPositions.RED_SHOOT_HIGH_GOAL, 180)
            .follow();

        appendages.shootRings();

        appendages.enableShooterWheel(false);

        drive.builder(FAST)
                .spline(FieldPositions.RED_GRAB_SECOND_GOAL, 0)
                .follow();

        appendages.setGoalLifterPosition(BotAppendages.GoalLifterPosition.MIDDLE);

        driveToZone(targetZone, -2);

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

        drive.followTrajectory(driveToLineBuilder.build());
    }

    void driveToZone(RingVision.TargetZone targetZone, double xOffset) {
        Pose2d offset = new Pose2d(xOffset, 0, 0);

        switch(targetZone) {
            case ZONE_A:
                drive.builder(FAST)
                        .spline(FieldPositions.RED_DELIVER_A.plus(offset), 0)
                        .follow();
                break;
            case ZONE_B:
                drive.builder(FAST)
                        .spline(FieldPositions.RED_OUTSIDE_AVOID_RINGS, 0)
                        .spline(FieldPositions.RED_DELIVER_B.plus(offset), 0)
                        .follow();
                break;
            case ZONE_C:
                drive.builder(FAST)
                        .spline(FieldPositions.RED_OUTSIDE_AVOID_RINGS, 0)
                        .spline(FieldPositions.RED_DELIVER_C.plus(offset), 0)
                        .follow();
                break;
        }
    }
}*/