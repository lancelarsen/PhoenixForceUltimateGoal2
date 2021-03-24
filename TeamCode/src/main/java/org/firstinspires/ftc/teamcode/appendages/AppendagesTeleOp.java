package org.firstinspires.ftc.teamcode.appendages;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class AppendagesTeleOp extends BotAppendages {
    private LinearOpMode opMode;

    private NanoClock nanoClock;
    private double startTime = -1;

    private boolean shooterTogglePrePressed = false;
    private boolean shooterEnabled = true;

    private boolean intakeTogglePrePressed = false;
    private boolean intakeEnabled = true;

    private boolean adjGoalLifterPrePressed = false;
    private boolean adjGoalLifterPosition = false;
    private GoalLifterPosition goalLifterPosition = GoalLifterPosition.DOWN;
    private boolean goalGrabberOpen = true;

    public AppendagesTeleOp(LinearOpMode opMode) {
        super(opMode.hardwareMap);
        this.opMode = opMode;

        nanoClock = NanoClock.system();
    }

    public boolean isAdjGoalLifterPosition() {
        return adjGoalLifterPosition;
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
        extendShooterArm(this.opMode.gamepad2.x);

        if (this.opMode.gamepad2.y && !shooterTogglePrePressed) {
            shooterEnabled = !shooterEnabled;
            shooterTogglePrePressed = true;
        }
        if (!this.opMode.gamepad2.y) {
            shooterTogglePrePressed = false;
        }

        setShooterTilterAngle(shooterEnabled ? ShooterAngle.SHOOTING : ShooterAngle.LOADING);
        enableShooterWheel(shooterEnabled);
    }

    public void commandIntake() {
        setIntakeDirection(this.opMode.gamepad2.a ? Direction.REVERSE : Direction.FORWARD);

        if (this.opMode.gamepad2.b && !intakeTogglePrePressed) {
            intakeEnabled = !intakeEnabled;
            intakeTogglePrePressed = true;
        }
        if (!this.opMode.gamepad2.b) {
            intakeTogglePrePressed = false;
        }

        enableIntake(intakeEnabled);
    }

    public void commandGoalGrabber() {
        if (this.opMode.gamepad1.right_stick_button && !adjGoalLifterPrePressed) {
            adjGoalLifterPosition = !adjGoalLifterPosition;
            adjGoalLifterPrePressed = true;

            if (adjGoalLifterPosition) {
                goalLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                goalLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                goalLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else if (!this.opMode.gamepad1.right_stick_button) {
            adjGoalLifterPrePressed = false;
        }

        if (adjGoalLifterPosition) {
            // Not sure why we need *-1, as we have motor reversed in HW class
            double rightStickY = this.opMode.gamepad1.right_stick_y * -1 * 0.5;
            if (Math.abs(rightStickY) < JOYSTICK_DEAD_ZONE) {
                rightStickY = 0;
            }

            goalLifter.setPower(rightStickY);
        } else {
            if (this.opMode.gamepad1.left_trigger > TRIGGER_PRESSED_THRESH) {
                goalLifterPosition = GoalLifterPosition.UP;
            } else if (this.opMode.gamepad1.right_trigger > TRIGGER_PRESSED_THRESH) {
                goalLifterPosition = GoalLifterPosition.DOWN;
            }

            setGoalLifterPosition(goalLifterPosition);
        }

        if (this.opMode.gamepad1.left_bumper) {
            goalGrabberOpen = false;
        } else if (this.opMode.gamepad1.right_bumper) {
            goalGrabberOpen = true;
        }

        openGoalGrabber(goalGrabberOpen);
    }
}
