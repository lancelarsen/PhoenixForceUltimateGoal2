package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.appendages.AppendagesTeleOp;
import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.drive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.drive.MecanumTeleOp;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.opmodes.teleop.util.GamepadUtils;

import static org.firstinspires.ftc.teamcode.appendages.BotAppendages.TRIGGER_PRESSED_THRESH;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.sleep;

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
//  - Left Trigger          - teleop-auto: shoot power shots
//  - Right Bumper          - teleop-auto: shoot high goal
//  - Right Trigger         -
//  - X                     - Shoot a Ring!
//  - B                     - Turn Ring Intake On/Off While Lifter Up
//  - Y                     - Turn Ring Shooter On/Off While Lifter Down
//  - A                     - Reverse Ring Intake Direction (While Pressed)
//----------------------------------------------------------------------

public class TeleOp {
    private volatile LinearOpMode opMode;
    private volatile AutoUtils.Alliance alliance;

    private volatile MecanumTeleOp mecanumTeleop;

    private volatile AppendagesTeleOp appendagesTeleop;

    private volatile GamepadUtils driverGamepad;
    private volatile Thread shootThread;

    private volatile boolean hasLocalizedRobot = false;
    private volatile int autoPowershotsShot = 0;

    private final static Pose2d BLUE_SIDE_INIT_POWERSHOT_POSE = new Pose2d(18, 54, Math.toRadians(0));
    private final static Pose2d BLUE_FRONT_INIT_POWERSHOT_POSE = new Pose2d(58, 11, Math.toRadians(0));
    private final static Pose2d RED_SIDE_INIT_POWERSHOT_POSE = new Pose2d(18, -59, Math.toRadians(0));
    private final static Pose2d RED_FRONT_INIT_POWERSHOT_POSE = new Pose2d(58, -11, Math.toRadians(0));

    private final static long AUTO_POWERSHOT_SHOOT_DELAY = 1000;
    private final static long AUTO_POWERSHOT_MOVE_DELAY = 1000;

    public TeleOp(LinearOpMode opMode, AutoUtils.Alliance alliance) {
        this.opMode = opMode;
        this.alliance = alliance;

        mecanumTeleop = new MecanumTeleOp(opMode);

        appendagesTeleop = new AppendagesTeleOp(opMode);

        driverGamepad = new GamepadUtils(opMode.gamepad1);
    }

    public void run() {
        appendagesTeleop.updateLights(alliance);
        appendagesTeleop.commandIntake();
        appendagesTeleop.commandShooter();
        appendagesTeleop.commandGoalGrabber();

        commandHighGoalShooting();
        commandFrontPowershotShooting();
        commandSidePowershotShooting();

        mecanumTeleop.enableTurning(!appendagesTeleop.isAdjGoalLifterPosition());
        mecanumTeleop.arcadeDrive();
    }

    private void commandHighGoalShooting() {
        if (opMode.gamepad2.right_trigger > TRIGGER_PRESSED_THRESH) {
            switch (alliance) {
                case RED:
                    mecanumTeleop.setPoseEstimate(FieldPositions.RL2);
                    shootHighGoal(FieldPositions.RTO);
                    break;
                case BLUE:
                    mecanumTeleop.setPoseEstimate(FieldPositions.BL2);
                    shootHighGoal(FieldPositions.BTO);
                    break;
            }
        }
    }

    private void shootHighGoal(Pose2d shootingPose) {
        Runnable shootTask = () -> {
            try {
                appendagesTeleop.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);

                Trajectory shootingTrajectory = line(shootingPose);

                mecanumTeleop.followTrajectoryAsync(shootingTrajectory);
                while (mecanumTeleop.isBusy()) {
                    mecanumTeleop.update();
                    if (Thread.interrupted()) {
                        mecanumTeleop.cancelFollowing();
                        throw new InterruptedException();
                    }
                }

                appendagesTeleop.shootRings();
            } catch (InterruptedException e) {};
        };

        shootThread = new Thread(shootTask);
        shootThread.start();

        waitForShootThread();
    }

    private void commandSidePowershotShooting() {
        if (opMode.gamepad2.left_trigger > TRIGGER_PRESSED_THRESH) {
            switch (alliance) {
                case RED:
                    mecanumTeleop.setPoseEstimate(FieldPositions.RL2);
                    shootPowershotsFromSide(FieldPositions.RTO_TELE_PS, FieldPositions.RTO_TELE_PSA);
                    break;
                case BLUE:
                    mecanumTeleop.setPoseEstimate(FieldPositions.BL2);
                    shootPowershotsFromSide(FieldPositions.P5, FieldPositions.P5A);
                    break;
            }
        }
    }

    private void shootPowershotsFromSide(Pose2d shootingPose, double turnAngles[]) {
        Runnable shootTask = () -> {
            try {
                appendagesTeleop.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
                Thread.sleep(2000);

                Trajectory shootingTrajectory = line(shootingPose);

                mecanumTeleop.followTrajectoryAsync(shootingTrajectory);
                while (mecanumTeleop.isBusy()) {
                    mecanumTeleop.update();
                    if (Thread.interrupted()) {
                        mecanumTeleop.cancelFollowing();
                        throw new InterruptedException();
                    }
                }

                for (int i = 0; i < 3; i++) {
                    appendagesTeleop.shootRings(1);
                    if (i == 2) break;
                    mecanumTeleop.turnAsync(Math.toRadians(turnAngles[i]));
                    mecanumTeleop.waitForIdle();
                }
            } catch (InterruptedException e) {};
        };

        shootThread = new Thread(shootTask);
        shootThread.start();

        waitForShootThread();
    }

    private void commandFrontPowershotShooting() {
        if (opMode.gamepad2.left_bumper) {
            if (!hasLocalizedRobot) return;

            switch (alliance) {
                case RED:
                    shootPowershotsFromFront(FieldPositions.P4, FieldPositions.P4A);
                    break;
                case BLUE:
                    shootPowershotsFromFront(FieldPositions.P2, FieldPositions.P2A);
                    break;
            }
        } else if (opMode.gamepad2.dpad_up) {
            hasLocalizedRobot = true;
            autoPowershotsShot = 0;

            switch (alliance) {
                case RED:
                    mecanumTeleop.setPoseEstimate(RED_FRONT_INIT_POWERSHOT_POSE);
                    shootPowershotsFromFront(FieldPositions.P4, FieldPositions.P4A);
                    break;
                case BLUE:
                    mecanumTeleop.setPoseEstimate(BLUE_FRONT_INIT_POWERSHOT_POSE);
                    shootPowershotsFromFront(FieldPositions.P2, FieldPositions.P2A);
                    break;
            }
        } else if (opMode.gamepad2.dpad_left && alliance == AutoUtils.Alliance.BLUE) {
            hasLocalizedRobot = true;
            autoPowershotsShot = 0;

            mecanumTeleop.setPoseEstimate(BLUE_SIDE_INIT_POWERSHOT_POSE);
            shootPowershotsFromFront(FieldPositions.P2, FieldPositions.P2A);
        } else if (opMode.gamepad2.dpad_right && alliance == AutoUtils.Alliance.RED) {
            hasLocalizedRobot = true;
            autoPowershotsShot = 0;

            mecanumTeleop.setPoseEstimate(RED_SIDE_INIT_POWERSHOT_POSE);
            shootPowershotsFromFront(FieldPositions.P4, FieldPositions.P4A);
        }
    }

    private void shootPowershotsFromFront(Pose2d shootingPose, double turnAngles[]) {
        Runnable shootTask = () -> {
            try {
                appendagesTeleop.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);

                Trajectory shootingTrajectory = line(shootingPose);

                mecanumTeleop.followTrajectoryAsync(shootingTrajectory);
                while (mecanumTeleop.isBusy()) {
                    mecanumTeleop.update();
                    if (Thread.interrupted()) {
                        mecanumTeleop.cancelFollowing();
                        throw new InterruptedException();
                    }
                }

                Thread.sleep(500);

                int previousPowershotsShot = autoPowershotsShot;

                for (int i = 0; i < 3; i++) {
                    // Don't shoot powershots already shot
                    if (i + 1 <= previousPowershotsShot) continue;

                    appendagesTeleop.shootRings(1);
                    autoPowershotsShot++;

                    Thread.sleep(AUTO_POWERSHOT_SHOOT_DELAY);

                    if (i == 2) break;

                    mecanumTeleop.turnAsync(Math.toRadians(turnAngles[i]));
                    mecanumTeleop.waitForIdle();
                    Thread.sleep(AUTO_POWERSHOT_MOVE_DELAY);
                }
            } catch (InterruptedException e) {};
        };

        shootThread = new Thread(shootTask);
        shootThread.start();

        waitForShootThread();
    }

    private void waitForShootThread() {
        while (shootThread.isAlive()) {
            //appendagesTeleop.updateLights(alliance);

            if (driverGamepad.areJoysticksActive() || driverGamepad.isDpadActive() || !opMode.opModeIsActive()) {
                shootThread.interrupt();
            }
        }
    }

    private Trajectory line(Pose2d pose) {
        return mecanumTeleop.trajectoryBuilder(mecanumTeleop.getPoseEstimate())
                .lineToLinearHeading(pose)
                .build();
    }

    private void follow(Trajectory trajectory) throws InterruptedException {
        mecanumTeleop.followTrajectoryAsync(trajectory);
        while (mecanumTeleop.isBusy()) {
            mecanumTeleop.update();
            if (Thread.interrupted()) {
                mecanumTeleop.cancelFollowing();
                throw new InterruptedException();
            }
        }
    }
}
