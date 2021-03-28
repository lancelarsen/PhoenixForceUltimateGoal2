package org.firstinspires.ftc.teamcode.appendages;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.appendages.utils.EncoderUtil;
import org.firstinspires.ftc.teamcode.appendages.utils.MapUtil;

@Config
public class BotAppendages {
    public final static double TRIGGER_PRESSED_THRESH = 0.5;
    public final static double JOYSTICK_DEAD_ZONE = 0.05;

    public final static double MIN_SHOOTER_TILTER_ANGLE = 0.1;
    public final static double MAX_SHOOTER_TILTER_ANGLE = 0.0;
    public final static double SHOOTER_TILTER_SHOOTING_ANGLE = 0.5;
    public final static double SHOOTER_TILTER_LOADING_ANGLE = 0.4;
    public final static double EXTENDED_SHOOTER_ARM_ANGLE = 0.5;
    public final static double RETRACTED_SHOOTER_ARM_ANGLE = 0.3;
    public final static double RING_SHOOTER_WHEEL_SPEED = 500;
    //public final static double RING_SHOOTER_WHEEL_SPEED = 0.78;

    public final static double INTAKE_ROLLER_SPEED = -1.0;
    public final static double INTAKE_ELEVATOR_SPEED = -1.0;

    public final static double GOAL_LIFTER_SPEED = 1;
    public final static double DOWN_GOAL_LIFTER_POSITION = 0;
    public final static double MIDDLE_GOAL_LIFTER_POSITION = 20;
    public final static double UP_GOAL_LIFTER_POSITION = 40; // In inches
    public final static double OPEN_GOAL_GRABBER_ANGLE = 0.6;
    public final static double CLOSED_GOAL_GRABBER_ANGLE = 0.18;
    public final static double OPEN_GOAL_LOCK_ANGLE = 0.5;
    public final static double CLOSED_GOAL_LOCK_ANGLE = 0;

    public final RevBlinkinLedDriver blinkin;
    public RevBlinkinLedDriver.BlinkinPattern blinkinPattern;

    public final Servo shooterTilter;
    public final Servo shooterArm;
    public final DcMotorEx shooterWheel;

    public final ColorSensor ringDetector;

    public final DcMotor intakeRoller;
    public final DcMotor intakeElevator;

    public final DcMotorEx goalLifter;
    public final Servo goalGrabber;
    public final Servo goalLock;
    private Thread goalLockThread;
    private boolean goalGrabberLastOpen = false;

    public boolean intakeEnabled;
    public Direction intakeDirection;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    public enum GoalLifterPosition {
        UP,
        MIDDLE,
        DOWN
    }

    public enum ShooterAngle {
        SHOOTING,
        LOADING
    }

    public BotAppendages(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        shooterTilter = hardwareMap.get(Servo.class, "shooterTilter");
        shooterArm = hardwareMap.get(Servo.class, "shooterArm");
        shooterWheel = hardwareMap.get(DcMotorEx.class, "shooterWheel");
        shooterWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        ringDetector = hardwareMap.get(ColorSensor.class, "ringDetector");

        intakeRoller = hardwareMap.get(DcMotor.class, "intakeRoller");
        //intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeElevator = hardwareMap.get(DcMotor.class, "intakeElevator");
        //intakeElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        goalLifter = hardwareMap.get(DcMotorEx.class, "goalGrabberLifter");
        goalLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goalLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goalGrabber = hardwareMap.get(Servo.class, "leftGrabberServo");
        goalLock = hardwareMap.get(Servo.class, "goalLock");
        goalLock.setDirection(Servo.Direction.REVERSE);
    }

    public void setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkin.setPattern(pattern);
        blinkinPattern = pattern;
    }

    public RevBlinkinLedDriver.BlinkinPattern getBlinkinPattern() {
        return blinkinPattern;
    }

    public void setShooterTilterAngle(ShooterAngle shooterAngle) {
        switch (shooterAngle) {
            case SHOOTING:
                shooterTilter.setPosition(SHOOTER_TILTER_SHOOTING_ANGLE);
                break;
            case LOADING:
                shooterTilter.setPosition(SHOOTER_TILTER_LOADING_ANGLE);
                break;
        }
    }

    public void enableShooterWheel(boolean enabled) {
        enableShooterWheel(enabled, RING_SHOOTER_WHEEL_SPEED);
    }

    public void enableShooterWheel(boolean enabled, double speed) {
        if (enabled) {
            shooterWheel.setPower(speed);
            //shooterWheel.setVelocity(speed);
        } else {
            shooterWheel.setVelocity(0);
        }
    }

    public void extendShooterArm(boolean extended) {
        if (extended) {
            shooterArm.setPosition(EXTENDED_SHOOTER_ARM_ANGLE);
        } else {
            shooterArm.setPosition(RETRACTED_SHOOTER_ARM_ANGLE);
        }
    }

    /*public boolean isRingLifterFull() {
        return
    }*/

    public void enableIntake(boolean enabled) {
        intakeEnabled = enabled;

        runIntake();
    }

    public void setIntakeDirection(Direction direction) {
        intakeDirection = direction;

        runIntake();
    }

    public void setGoalLifterPosition(GoalLifterPosition position) {
        double lifterPosition = DOWN_GOAL_LIFTER_POSITION;
        switch (position) {
            case UP:
                lifterPosition = UP_GOAL_LIFTER_POSITION;
                break;
            case MIDDLE:
                lifterPosition = MIDDLE_GOAL_LIFTER_POSITION;
                break;
        }

        goalLifter.setTargetPosition(EncoderUtil.inchesToTicks(EncoderUtil.Motor.GOBILDA_5202, lifterPosition));
        goalLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goalLifter.setPower(GOAL_LIFTER_SPEED);
    }

    public void openGoalGrabber(boolean open) {
        if (goalGrabberLastOpen == open) return;
        goalGrabberLastOpen = open;

        if (goalLockThread != null) goalLockThread.interrupt();

        Runnable openTask = () -> {
            try {
                if (open) {
                    goalLock.setPosition(OPEN_GOAL_LOCK_ANGLE);
                    Thread.sleep(100);
                    goalGrabber.setPosition(OPEN_GOAL_GRABBER_ANGLE);
                } else {
                    goalGrabber.setPosition(CLOSED_GOAL_GRABBER_ANGLE);
                    Thread.sleep(400);
                    goalLock.setPosition(CLOSED_GOAL_LOCK_ANGLE);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        };

        goalLockThread = new Thread(openTask);
        goalLockThread.start();
    }

    private void runIntake() {
        double intakeRollerSpeed = 0;
        double intakeElevatorSpeed = 0;

        if (intakeEnabled) {
            double direction = intakeDirection == Direction.FORWARD ? 1 : -1;

            intakeRollerSpeed = BotAppendages.INTAKE_ROLLER_SPEED * direction;
            intakeElevatorSpeed = BotAppendages.INTAKE_ELEVATOR_SPEED * direction;
        }

        intakeRoller.setPower(intakeRollerSpeed);
        intakeElevator.setPower(intakeElevatorSpeed);
    }
}
