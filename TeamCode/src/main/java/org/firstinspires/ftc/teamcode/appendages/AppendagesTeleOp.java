package org.firstinspires.ftc.teamcode.appendages;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.teleop.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.opmodes.teleop.util.GamepadUtils;

public class AppendagesTeleOp extends BotAppendages {
    private LinearOpMode opMode;

    private NanoClock nanoClock;
    private double startTime = -1;

    private ButtonToggle shooterToggle = new ButtonToggle();
    private boolean shootingInterruptedforMovement;
    private Thread shootThread;
    private GamepadUtils driverGamepad;

    private ButtonToggle intakeToggle = new ButtonToggle();

    private ButtonToggle goalLifterAdjToggle = new ButtonToggle();
    private boolean preAdjGoalLifter;
    private GoalLifterPosition goalLifterPosition = GoalLifterPosition.DOWN;
    private boolean goalGrabberOpen = true;

    private final Runnable shootTask = () -> {
        while (opMode.opModeIsActive()) {
            shootRings(1);
        }
    };

    public AppendagesTeleOp(LinearOpMode opMode) {
        super(opMode.hardwareMap);
        this.opMode = opMode;

        nanoClock = NanoClock.system();

        driverGamepad = new GamepadUtils(opMode.gamepad1);
    }

    public boolean isAdjGoalLifterPosition() {
        return goalLifterAdjToggle.isActive();
    }

    public void updateLights() {
        if (startTime == -1) {
            startTime = nanoClock.seconds();
        }
        double activeTime = nanoClock.seconds() - startTime;

        if (80 < activeTime && activeTime < 90) {
            setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
        } else {
            setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        }
    }

    public void commandShooter() {
        if (opMode.gamepad2.x && !shootingInterruptedforMovement && !driverGamepad.areJoysticksActive() && !driverGamepad.isDpadActive()) {
            if (shootThread == null || !shootThread.isAlive()) {
                shootThread = new Thread(shootTask);
                shootThread.start();
            }
        } else {
            if (shootThread != null) shootThread.interrupt();
            extendShooterArm(false);

            if (driverGamepad.areJoysticksActive() || driverGamepad.isDpadActive()) {
                shootingInterruptedforMovement = true;
            }

            if (!opMode.gamepad2.x) {
                shootingInterruptedforMovement = false;
            }
        }

        shooterToggle.update(opMode.gamepad2.y);

        setShooterTilterAngle(shooterToggle.isActive() ? ShooterAngle.SHOOTING : ShooterAngle.LOADING);
        setShooterSpeed(shooterToggle.isActive() ? ShooterSpeed.HIGH_GOAL : ShooterSpeed.OFF);
    }

    public void commandIntake() {
        setIntakeDirection(opMode.gamepad2.a ? Direction.REVERSE : Direction.FORWARD);

        intakeToggle.update(opMode.gamepad2.b);
        enableIntake(intakeToggle.isActive());
    }

    public void commandGoalGrabber() {
        goalLifterAdjToggle.update(opMode.gamepad1.right_stick_button);
        if (goalLifterAdjToggle.isActive() != preAdjGoalLifter) {
            if (goalLifterAdjToggle.isActive()) {
                goalLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                goalLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                goalLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        preAdjGoalLifter = goalLifterAdjToggle.isActive();

        if (goalLifterAdjToggle.isActive()) {
            // Not sure why we need *-1, as we have motor reversed in HW class
            double rightStickY = opMode.gamepad1.right_stick_y * -1 * 0.5;
            if (Math.abs(rightStickY) < JOYSTICK_DEAD_ZONE) {
                rightStickY = 0;
            }

            goalLifter.setPower(rightStickY);
        } else {
            if (opMode.gamepad1.left_trigger > TRIGGER_PRESSED_THRESH) {
                goalLifterPosition = GoalLifterPosition.UP;
            } else if (opMode.gamepad1.right_trigger > TRIGGER_PRESSED_THRESH) {
                goalLifterPosition = GoalLifterPosition.DOWN;
            }

            setGoalLifterPosition(goalLifterPosition);
        }

        if (opMode.gamepad1.left_bumper) {
            goalGrabberOpen = false;
        } else if (opMode.gamepad1.right_bumper) {
            goalGrabberOpen = true;
        }

        openGoalGrabber(goalGrabberOpen);
    }
}
