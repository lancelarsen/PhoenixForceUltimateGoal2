package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    private LinearOpMode opMode;
    private AutoUtils.Alliance alliance;

    private volatile AppendagesTeleOp appendages;
    private volatile MecanumTeleOp mecanum;

    private GamepadUtils driverGamepad;
    private Thread shootThread;

    private boolean hasLocalizedRobot = false;
    private int autoPowershotsShot = 0;

    private final static Pose2d BLUE_SIDE_INIT_POWERSHOT_POSE = new Pose2d(18, 54, Math.toRadians(0));
    private final static Pose2d BLUE_FRONT_INIT_POWERSHOT_POSE = new Pose2d(58, 11, Math.toRadians(0));
    private final static Pose2d RED_SIDE_INIT_POWERSHOT_POSE = new Pose2d(18, -59, Math.toRadians(0));
    private final static Pose2d RED_FRONT_INIT_POWERSHOT_POSE = new Pose2d(58, -11, Math.toRadians(0));

    private final static long AUTO_POWERSHOT_SHOOT_DELAY = 1000;
    private final static long AUTO_POWERSHOT_MOVE_DELAY = 1000;

    public MecanumAutonomous drive;

    public TeleOp(LinearOpMode opMode, AutoUtils.Alliance alliance) {
        this.opMode = opMode;
        this.alliance = alliance;

        appendages = new AppendagesTeleOp(opMode);
        mecanum = new MecanumTeleOp(opMode);
        driverGamepad = new GamepadUtils(opMode.gamepad1);

        drive = new MecanumAutonomous(opMode);
    }

    public void run() {
        appendages.updateLights(alliance);
        appendages.commandIntake();
        appendages.commandShooter();
        appendages.commandGoalGrabber();

        //commandPowershotShooting();
        teleopShootHighGoalFromSides();
        teleopShootPowerShotsFromSides();

        mecanum.enableTurning(!appendages.isAdjGoalLifterPosition());
        mecanum.arcadeDrive();
    }

    private void teleopShootHighGoalFromSides() {
        if (opMode.gamepad2.right_trigger > TRIGGER_PRESSED_THRESH) {
            switch (alliance) {
                case RED:
                    drive.setCurrentPosition(FieldPositions.RSA);
                    appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
                    drive.line(FieldPositions.RSB);
                    sleep(1000);
                    appendages.shootRings();
                    break;
                case BLUE:
                    drive.setCurrentPosition(FieldPositions.BSA);
                    appendages.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
                    drive.line(FieldPositions.BSB);
                    sleep(1000);
                    appendages.shootRings();
                    break;
            }
        }
    }

    private void teleopShootPowerShotsFromSides() {
        if (opMode.gamepad2.left_trigger > TRIGGER_PRESSED_THRESH) {
            switch (alliance) {
                case RED:
                    drive.setCurrentPosition(FieldPositions.RSA);
                    shootPowershotsFromSide(FieldPositions.P6, FieldPositions.P6A);
                    break;
                case BLUE:
                    drive.setCurrentPosition(FieldPositions.BSA);
                    shootPowershotsFromSide(FieldPositions.P5, FieldPositions.P5A);
                    break;
            }
        }
    }

    private void shootPowershotsFromSide(Pose2d shootingPose, double turnAngles[]) {
        appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
        drive.line(shootingPose);
        for (int i = 0; i < 3; i++) {
            appendages.shootRings(1);
            sleep(500);

            if (i == 2) break;
            drive.turnLeft(turnAngles[i]);
            sleep(500);
        }
    }

    private void commandPowershotShooting() {
        if (opMode.gamepad2.left_bumper) {
            if (!hasLocalizedRobot) return;

            switch (alliance) {
                case RED:
                    shootPowershots(FieldPositions.P4, FieldPositions.P4A);
                    break;
                case BLUE:
                    shootPowershots(FieldPositions.P2, FieldPositions.P2A);
                    break;
            }
        } else if (opMode.gamepad2.dpad_up) {
            hasLocalizedRobot = true;
            autoPowershotsShot = 0;

            switch (alliance) {
                case RED:
                    mecanum.setPoseEstimate(RED_FRONT_INIT_POWERSHOT_POSE);
                    shootPowershots(FieldPositions.P4, FieldPositions.P4A);
                    break;
                case BLUE:
                    mecanum.setPoseEstimate(BLUE_FRONT_INIT_POWERSHOT_POSE);
                    shootPowershots(FieldPositions.P2, FieldPositions.P2A);
                    break;
            }
        } else if (opMode.gamepad2.dpad_left && alliance == AutoUtils.Alliance.BLUE) {
            hasLocalizedRobot = true;
            autoPowershotsShot = 0;

            mecanum.setPoseEstimate(BLUE_SIDE_INIT_POWERSHOT_POSE);
            shootPowershots(FieldPositions.P2, FieldPositions.P2A);
        } else if (opMode.gamepad2.dpad_right && alliance == AutoUtils.Alliance.RED) {
            hasLocalizedRobot = true;
            autoPowershotsShot = 0;

            mecanum.setPoseEstimate(RED_SIDE_INIT_POWERSHOT_POSE);
            shootPowershots(FieldPositions.P4, FieldPositions.P4A);
        }
    }

    private void shootPowershots(Pose2d shootingPose, double turnAngles[]) {
        Runnable shootTask = () -> {
            try {
                appendages.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);

                Trajectory shootingTrajectory = mecanum.trajectoryBuilder(mecanum.getPoseEstimate())
                        .lineToLinearHeading(shootingPose)
                        .build();

                mecanum.followTrajectoryAsync(shootingTrajectory);
                while (mecanum.isBusy()) {
                    mecanum.update();
                    if (Thread.interrupted()) {
                        mecanum.cancelFollowing();
                        return;
                    }
                }

                Thread.sleep(500);

                int previousPowershotsShot = autoPowershotsShot;

                for (int i = 0; i < 3; i++) {
                    // Don't shoot powershots already shot
                    if (i + 1 <= previousPowershotsShot) continue;

                    appendages.shootRings(1);
                    autoPowershotsShot++;

                    Thread.sleep(AUTO_POWERSHOT_SHOOT_DELAY);

                    if (i == 2) break;

                    mecanum.turnAsync(Math.toRadians(turnAngles[i]));
                    mecanum.waitForIdle();
                    Thread.sleep(AUTO_POWERSHOT_MOVE_DELAY);
                }
            } catch (InterruptedException e) {};
        };

        shootThread = new Thread(shootTask);
        shootThread.start();

        while (shootThread.isAlive()) {
            if (driverGamepad.areJoysticksActive() || driverGamepad.isDpadActive() || !opMode.opModeIsActive()) {
                shootThread.interrupt();
            }
        }
    }
}
