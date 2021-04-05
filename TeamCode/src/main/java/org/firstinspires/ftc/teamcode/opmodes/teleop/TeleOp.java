package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.appendages.AppendagesTeleOp;
import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.drive.MecanumTeleOp;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.opmodes.teleop.util.GamepadUtils;

import java.util.concurrent.TimeUnit;

//----------------------------------------------------------------------
// Gamepad 1
//  - Left Stick            - Arcade Drive Movement
//  - Right Stick           - Arcade Turning (Push: Realign Goal Lifter)
//  - Dpad Up               - Drive Straight Forward
//  - Dpad Down             - Drive Straight Backward
//  - Dpad Right            - Drive Strafe Right
//  - Dpad Left             - Drive Strafe Left
//  - Left Bumper           - Goal Grabber Open
//  - Left Trigger          - Goal Lifter Up
//  - Right Bumper          - Goal Grabber Closed
//  - Right Trigger         - Goal Lifter Down
//  - X                     - Auto Shoot Powershots
//  - Y                     - Speed Fast
//  - B                     - Speed Slow
//  - A                     -
//----------------------------------------------------------------------
// Gamepad 2
//  - Left Stick            - Tilt Ring Shooter
//  - Right Stick           - (Push: Realign Ring Lifter)
//  - Dpad Up               - Ring Lifter Up
//  - Dpad Down             - Ring Lifter Down
//  - Dpad Right            -
//  - Dpad Left             -
//  - Left Bumper           -
//  - Left Trigger          -
//  - Right Bumper          -
//  - Right Trigger         -
//  - X                     - Shoot a Ring!
//  - B                     - Turn Ring Intake On/Off While Lifter Up
//  - Y                     - Turn Ring Shooter On/Off While Lifter Down
//  - A                     - Reverse Ring Intake Direction (While Pressed)
//----------------------------------------------------------------------

public class TeleOp {
    LinearOpMode opMode;
    AutoUtils.Alliance alliance;

    AppendagesTeleOp appendages;
    MecanumTeleOp mecanumTeleOp;
    MecanumAutonomous mecanumAutonomous;

    GamepadUtils driverGamepad;
    Thread shootThread;

    final static Pose2d BLUE_SIDE_INIT_POWERSHOT_POSE = new Pose2d(10, -61, Math.toRadians(0));
    final static Pose2d BLUE_FRONT_INIT_POWERSHOT_POSE = new Pose2d(58, 11, Math.toRadians(0));
    final static Pose2d RED_SIDE_INIT_POWERSHOT_POSE = new Pose2d(10, -61, Math.toRadians(0));
    final static Pose2d RED_FRONT_INIT_POWERSHOT_POSE = new Pose2d(58, -11, Math.toRadians(0));

    final static long AUTO_POWERSHOT_SHOOT_DELAY = 1000;
    final static long AUTO_POWERSHOT_MOVE_DELAY = 1000;

    public TeleOp(LinearOpMode opMode, AutoUtils.Alliance alliance) {
        this.opMode = opMode;
        this.alliance = alliance;

        appendages = new AppendagesTeleOp(opMode);

        mecanumTeleOp = new MecanumTeleOp(opMode);
        mecanumAutonomous = new MecanumAutonomous(opMode);

        driverGamepad = new GamepadUtils(opMode.gamepad1);
    }

    public void run() {
        appendages.updateLights(alliance);
        appendages.commandIntake();
        appendages.commandShooter();
        appendages.commandGoalGrabber();

        commandPowershotShooting();

        mecanumTeleOp.enableTurning(!appendages.isAdjGoalLifterPosition());
        mecanumTeleOp.arcadeDrive();
    }

    private void commandPowershotShooting() {
        if (opMode.gamepad2.left_bumper) {
            switch (alliance) {
                case BLUE:
                    shootPowershots(FieldPositions.P2, FieldPositions.P2A);
                    break;
                case RED:
                    shootPowershots(FieldPositions.P4, FieldPositions.P4A);
                    break;
            }
        } else if (opMode.gamepad2.dpad_up) {
            switch (alliance) {
                case BLUE:
                    mecanumAutonomous.setCurrentPosition(BLUE_FRONT_INIT_POWERSHOT_POSE);
                    shootPowershots(FieldPositions.P2, FieldPositions.P2A);
                    break;
                case RED:
                    mecanumAutonomous.setCurrentPosition(RED_FRONT_INIT_POWERSHOT_POSE);
                    shootPowershots(FieldPositions.P4, FieldPositions.P4A);
                    break;
            }
        } else if (opMode.gamepad2.dpad_left && alliance == AutoUtils.Alliance.BLUE) {
            mecanumAutonomous.setCurrentPosition(BLUE_SIDE_INIT_POWERSHOT_POSE);
            shootPowershots(FieldPositions.P2, FieldPositions.P2A);
        } else if (opMode.gamepad2.dpad_right && alliance == AutoUtils.Alliance.RED) {
            mecanumAutonomous.setCurrentPosition(RED_SIDE_INIT_POWERSHOT_POSE);
            shootPowershots(FieldPositions.P4, FieldPositions.P4A);
        }

        //appendages.shooterOff();
    }

    private void shootPowershots(Pose2d shootingPose, double turnAngles[]/*, int ringsToShoot*/) {
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);

        Trajectory shootingTrajectory =  mecanumAutonomous.trajectoryBuilder(mecanumAutonomous.getPoseEstimate())
                .lineToLinearHeading(shootingPose)
                .build();

        mecanumAutonomous.followTrajectoryAsync(shootingTrajectory);
        while (mecanumAutonomous.isBusy()) {
            mecanumAutonomous.update();
            if (!canContinueShooting()) {
                mecanumAutonomous.cancelFollowing();
                return;
            }
        }
        mecanumAutonomous.setCurrentPosition(shootingTrajectory.end());

        Deadline deadline = new Deadline(500, TimeUnit.MILLISECONDS);
        while (!deadline.hasExpired()) {
            if (!canContinueShooting()) {
                return;
            }
        }

        for (int i = 0; i < 3/* - ringsToShoot*/; i++) {
            appendages.shootRings(1);

            deadline = new Deadline(AUTO_POWERSHOT_SHOOT_DELAY, TimeUnit.MILLISECONDS);
            while (!deadline.hasExpired()) {
                if (!canContinueShooting()) {
                    return;
                }
            }

            if (i == 3/* - ringsToShoot*/) break;

            mecanumAutonomous.turnLeft(turnAngles[i]);

            deadline = new Deadline(AUTO_POWERSHOT_MOVE_DELAY, TimeUnit.MILLISECONDS);
            while (!deadline.hasExpired()) {
                if (!canContinueShooting()) {
                    return;
                }
            }
        }
    }

    boolean canContinueShooting() {
        return !driverGamepad.areJoysticksActive() && !driverGamepad.isDpadActive() && opMode.opModeIsActive();
    }
}
