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
    private LinearOpMode opMode;
    private AutoUtils.Alliance alliance;

    public volatile MecanumAutonomous mecanumAuto;
    private volatile MecanumTeleOp mecanumTeleop;

    private volatile AppendagesTeleOp appendagesTeleop;
    private volatile AppendagesAutonomous appendagesAuto;

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

    public TeleOp(LinearOpMode opMode, AutoUtils.Alliance alliance) {
        this.opMode = opMode;
        this.alliance = alliance;

        mecanumTeleop = new MecanumTeleOp(opMode);
        mecanumAuto = new MecanumAutonomous(opMode);

        appendagesTeleop = new AppendagesTeleOp(opMode);
        appendagesAuto = new AppendagesAutonomous(opMode);

        driverGamepad = new GamepadUtils(opMode.gamepad1);
    }

    public void run() {
        appendagesTeleop.updateLights(alliance);
        appendagesTeleop.commandIntake();
        appendagesTeleop.commandShooter();
        appendagesTeleop.commandGoalGrabber();

        commandPowershotShooting();
        shootHighGoalFromSide();
        shootPowerShotsFromSide();

        mecanumTeleop.enableTurning(!appendagesTeleop.isAdjGoalLifterPosition());
        mecanumTeleop.arcadeDrive();
    }

    private void shootHighGoalFromSide() {
        if (opMode.gamepad2.right_trigger > TRIGGER_PRESSED_THRESH) {
            switch (alliance) {
                case RED:
                    mecanumAuto.setCurrentPosition(FieldPositions.RL2);
                    appendagesTeleop.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
                    mecanumAuto.line(FieldPositions.RTO);
                    sleep(1000);
                    appendagesTeleop.shootRings();
                    break;
                case BLUE:
                    mecanumAuto.setCurrentPosition(FieldPositions.BL2);
                    appendagesTeleop.setShooterSpeed(BotAppendages.ShooterSpeed.HIGH_GOAL);
                    mecanumAuto.line(FieldPositions.BTO);
                    sleep(1000);
                    appendagesTeleop.shootRings();
                    break;
            }
        }
    }

    private void shootPowerShotsFromSide() {
        if (opMode.gamepad2.left_trigger > TRIGGER_PRESSED_THRESH) {
            switch (alliance) {
                case RED:
                    appendagesAuto.ringIntakeStop();
                    mecanumAuto.setCurrentPosition(FieldPositions.RL2);
                    shootPowershotsFromSide(FieldPositions.RTO_TELE_PS, FieldPositions.RTO_TELE_PSA);
                    appendagesAuto.ringIntakeStart();
                    break;
                case BLUE:
                    mecanumAuto.setCurrentPosition(FieldPositions.BL2);
                    shootPowershotsFromSide(FieldPositions.P5, FieldPositions.P5A);
                    break;
            }
        }
    }

     private void shootPowershotsFromSide(Pose2d shootingPose, double turnAngles[]) {
        appendagesTeleop.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);
        sleep(2000);
        mecanumAuto.line(shootingPose);
        for (int i = 0; i < 3; i++) {
            appendagesTeleop.shootRings(1);
            if (i == 2) break;
            mecanumAuto.turnLeft(turnAngles[i]);
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
                    mecanumTeleop.setPoseEstimate(RED_FRONT_INIT_POWERSHOT_POSE);
                    shootPowershots(FieldPositions.P4, FieldPositions.P4A);
                    break;
                case BLUE:
                    mecanumTeleop.setPoseEstimate(BLUE_FRONT_INIT_POWERSHOT_POSE);
                    shootPowershots(FieldPositions.P2, FieldPositions.P2A);
                    break;
            }
        } else if (opMode.gamepad2.dpad_left && alliance == AutoUtils.Alliance.BLUE) {
            hasLocalizedRobot = true;
            autoPowershotsShot = 0;

            mecanumTeleop.setPoseEstimate(BLUE_SIDE_INIT_POWERSHOT_POSE);
            shootPowershots(FieldPositions.P2, FieldPositions.P2A);
        } else if (opMode.gamepad2.dpad_right && alliance == AutoUtils.Alliance.RED) {
            hasLocalizedRobot = true;
            autoPowershotsShot = 0;

            mecanumTeleop.setPoseEstimate(RED_SIDE_INIT_POWERSHOT_POSE);
            shootPowershots(FieldPositions.P4, FieldPositions.P4A);
        }
    }

    private void shootPowershots(Pose2d shootingPose, double turnAngles[]) {
        Runnable shootTask = () -> {
            try {
                appendagesTeleop.setShooterSpeed(BotAppendages.ShooterSpeed.POWERSHOTS);

                Trajectory shootingTrajectory = mecanumTeleop.trajectoryBuilder(mecanumTeleop.getPoseEstimate())
                        .lineToLinearHeading(shootingPose)
                        .build();

                mecanumTeleop.followTrajectoryAsync(shootingTrajectory);
                while (mecanumTeleop.isBusy()) {
                    mecanumTeleop.update();
                    if (Thread.interrupted()) {
                        mecanumTeleop.cancelFollowing();
                        return;
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

        while (shootThread.isAlive()) {
            if (driverGamepad.areJoysticksActive() || driverGamepad.isDpadActive() || !opMode.opModeIsActive()) {
                shootThread.interrupt();
            }
        }
    }
}
