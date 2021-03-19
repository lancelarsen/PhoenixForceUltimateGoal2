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
    private boolean shooterEnabled = false;

    private boolean adjRingLifterPrePressed = false;
    private boolean adjRingLifterPosition = false;
    private RingLifterPosition ringLifterPosition = RingLifterPosition.DOWN;

    private boolean intakeTogglePrePressed = false;
    private boolean intakeEnabled = false;

    private boolean leftWingTogglePrePressed = false;
    private boolean leftWingExtended = false;
    private boolean rightWingTogglePrePressed = false;
    private boolean rightWingExtended = false;

    private boolean adjGoalLifterPrePressed = false;
    private boolean adjGoalLifterPosition = false;
    private GoalLifterPosition goalLifterPosition = GoalLifterPosition.DOWN;
    private boolean goalGrabberOpen = true;

    public AppendagesTeleOp(LinearOpMode opMode) {
        super(opMode.hardwareMap);
        this.opMode = opMode;

        nanoClock = NanoClock.system();

        // Do safe wing retract
        // (open wings, retract feathers, retract wings)
        setInitialLeftWingPos(true);
        setInitialRightWingPos(true);
        extendLeftWing(false);
        extendRightWing(false);
    }

    public boolean isAdjRingLifterPosition() {
        return adjRingLifterPosition;
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
        if (this.opMode.gamepad2.right_stick_button && !adjRingLifterPrePressed) {
            adjRingLifterPosition = !adjRingLifterPosition;
            adjRingLifterPrePressed = true;

            if (adjRingLifterPosition) {
                ringLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                ringLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ringLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        if (!this.opMode.gamepad2.right_stick_button) {
            adjRingLifterPrePressed = false;
        }

        if (adjRingLifterPosition) {
            // Not sure why we need *-1, as we have motor reversed in HW class
            double rightStickY = this.opMode.gamepad2.right_stick_y * -1 * 0.5;
            if (Math.abs(rightStickY) < JOYSTICK_DEAD_ZONE) {
                rightStickY = 0;
            }

            ringLifter.setPower(rightStickY);
        } else {
            if (this.opMode.gamepad2.dpad_up) {
                ringLifterPosition = RingLifterPosition.UP;
            } else if (this.opMode.gamepad2.dpad_down) {
                ringLifterPosition = RingLifterPosition.DOWN;
            }

            setRingLifterPosition(ringLifterPosition);
        }

        double leftStickY = this.opMode.gamepad2.left_stick_y;
        if (Math.abs(leftStickY) < JOYSTICK_DEAD_ZONE) {
            leftStickY = 0;
        }

        setShooterTilterAngle(leftStickY);

        extendShooterArm(this.opMode.gamepad2.x);

        if (this.opMode.gamepad2.y && !shooterTogglePrePressed) {
            shooterEnabled = !shooterEnabled;
            shooterTogglePrePressed = true;
        }
        if (!this.opMode.gamepad2.y) {
            shooterTogglePrePressed = false;
        }

        if (ringLifterPosition == RingLifterPosition.UP) {
            enableShooterWheel(true);
        } else {
            enableShooterWheel(shooterEnabled);
        }
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

        if (ringLifterPosition == RingLifterPosition.DOWN) {
            enableIntake(true);
        } else {
            enableIntake(intakeEnabled);
        }
    }

    public void commandWings() {
        if (this.opMode.gamepad2.left_bumper && !leftWingTogglePrePressed) {
            leftWingExtended = !leftWingExtended;
            leftWingTogglePrePressed = true;
        }
        if (!this.opMode.gamepad2.left_bumper) {
            leftWingTogglePrePressed = false;
        }

        if (this.opMode.gamepad2.right_bumper && !rightWingTogglePrePressed) {
            rightWingExtended = !rightWingExtended;
            rightWingTogglePrePressed = true;
        }
        if (!this.opMode.gamepad2.right_bumper) {
            rightWingTogglePrePressed = false;
        }

        extendLeftWing(leftWingExtended);
        extendRightWing(rightWingExtended);

        sweepLeftWing(this.opMode.gamepad2.left_trigger > TRIGGER_PRESSED_THRESH);
        sweepRightWing(this.opMode.gamepad2.right_trigger > TRIGGER_PRESSED_THRESH);
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

            if (ringLifterPosition == RingLifterPosition.UP) {
                setGoalLifterPosition(GoalLifterPosition.UP);
                setCameraPosition(CameraPosition.TOWARDS_GOAL);
            } else {
                setGoalLifterPosition(goalLifterPosition);
            }
        }

        if (this.opMode.gamepad1.left_bumper) {
            goalGrabberOpen = false;
        } else if (this.opMode.gamepad1.right_bumper) {
            goalGrabberOpen = true;
        }

        openGoalGrabber(goalGrabberOpen);
    }
}
