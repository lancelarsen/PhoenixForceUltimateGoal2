package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.MecanumAuto;
import org.firstinspires.ftc.teamcode.vision.RingVision;

import static org.firstinspires.ftc.teamcode.drive.MecanumAuto.Speed.FAST;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.Alliance;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.sleep;

public class FullAuto {
    private enum TargetZone {
        ZONE_A,
        ZONE_B,
        ZONE_C,
    }

    private LinearOpMode opMode;
    private AutoUtils.StartingPosition startingPosition;
    private Alliance alliance;

    private MecanumAuto drive;
    private AppendagesAutonomous appendages;
    private RingVision ringVision;

    public FullAuto(LinearOpMode opMode, Alliance alliance, AutoUtils.StartingPosition startingPosition) {
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
        RingVision.RingCount ringCount = ringVision.getRingCount();
        ringVision.setViewportPaused(true);

        TargetZone targetZone = TargetZone.ZONE_C;
        switch (ringCount) {
            case ZERO:
                targetZone = TargetZone.ZONE_A; break;
            case ONE:
                targetZone = TargetZone.ZONE_B; break;
        }

        opMode.telemetry.addData("Target zone", targetZone);
        opMode.telemetry.update();

        drive.builder(FAST)
                .spline(FieldPositions.RED_OUTSIDE_GRAB_GOAL, 0)
                .follow();

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

        /*TrajectoryBuilder driveToLineBuilder = drive.trajectoryBuilder(endPosition, false);
        if (targetZone == TargetZone.ZONE_A) {
            driveToLineBuilder.splineToConstantHeading(new Vector2d(-8, alliance == Alliance.RED ? -50 : 50), Math.toRadians(90));
        }
        driveToLineBuilder.splineToConstantHeading(new Vector2d(12, alliance == Alliance.RED ? -40 : 40), Math.toRadians(0));

        drive.followTrajectory(driveToLineBuilder.build());*/
    }

    void driveToZone(TargetZone targetZone, double xOffset) {
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
}