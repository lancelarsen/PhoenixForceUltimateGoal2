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

    public final static double CAMERA_ANGLE_TOWARDS_RINGS = 0.98;
    public final static double CAMERA_ANGLE_TOWARDS_GOAL = 0.25;

    public final static double MIN_SHOOTER_TILTER_ANGLE = 0.1;
    public final static double MAX_SHOOTER_TILTER_ANGLE = 0.0;
    public final static double EXTENDED_SHOOTER_ARM_ANGLE = 0.6;
    public final static double RETRACTED_SHOOTER_ARM_ANGLE = 0.75;
    public final static double RING_SHOOTER_WHEEL_SPEED = 1700;

    public final static double RING_LIFTER_SPEED = 1.0;
    public final static double DOWN_RING_LIFTER_POSITION = 0;
    public final static double UP_RING_LIFTER_POSITION = 7.25; // In inches

    public final static double EXTENDED_INTAKE_DEPLOYER_ANGLE = 0.0;
    public final static double RETRACTED_INTAKE_DEPLOYER_ANGLE = 0.6;
    public final static double LARGE_INTAKE_ROLLER_SPEED = 1.0;
    public final static double SMALL_INTAKE_ROLLER_SPEED = 1.0;
    public final static double SIDE_INTAKE_ROLLER_SPEED = 1.0;

    public final static double SWEEP_LEFT_WING_ANGLE = 1;
    public final static double SWEEP_RIGHT_WING_ANGLE = 0.26;
    public final static double SUPER_OPEN_LEFT_WING_ANGLE = 0.8;
    public final static double SUPER_OPEN_RIGHT_WING_ANGLE = 0.55;
    public final static double OPEN_LEFT_WING_ANGLE = 0.7;
    public final static double OPEN_RIGHT_WING_ANGLE = 0.64;
    public final static double CLOSED_LEFT_WING_ANGLE = 0;
    public final static double CLOSED_RIGHT_WING_ANGLE = 1;

    public final static double EXTENDED_LEFT_WING_ANGLE = 0.3;
    public final static double EXTENDED_RIGHT_WING_ANGLE = 0.57;
    public final static double RETRACTED_LEFT_WING_ANGLE = 1;
    public final static double RETRACTED_RIGHT_WING_ANGLE = 0;

    public final static int OPEN_WING_DELAY = 2000;
    public final static int CLOSE_WING_DELAY = 750;
    public final static int EXTEND_WING_DELAY = 1000;

    public final static double GOAL_LIFTER_SPEED = 1;
    public final static double DOWN_GOAL_LIFTER_POSITION = 0;
    public final static double MIDDLE_GOAL_LIFTER_POSITION = 20;
    public final static double UP_GOAL_LIFTER_POSITION = 40; // In inches
    public final static double OPEN_GOAL_GRABBER_ANGLE = 0.18;
    public final static double CLOSED_GOAL_GRABBER_ANGLE = 0.72;
    public final static double OPEN_GOAL_LOCK_ANGLE = 0.8;
    public final static double CLOSED_GOAL_LOCK_ANGLE = 0;

    public final RevBlinkinLedDriver blinkin;
    public RevBlinkinLedDriver.BlinkinPattern blinkinPattern;

    public final Servo cameraTurner;

    public final Servo shooterTilter;
    public final Servo shooterArm;
    public final DcMotorEx shooterWheel;

    public final ColorSensor ringDetector;
    public final DcMotorEx ringLifter;

    public final CRServo sideIntakeWheel;
    public final DcMotor largeIntakeRoller;
    public final CRServo smallIntakeRoller;

    public final Servo leftWingOpener;
    public final Servo rightWingOpener;
    public final Servo leftWingExtender;
    public final Servo rightWingExtender;
    private Thread leftExtendThread;
    private boolean leftWingLastExtended = false;
    private volatile boolean leftWingFullyOpened = false;
    private Thread rightExtendThread;
    private boolean rightWingLastExtended = false;
    private volatile boolean rightWingFullyOpened = false;

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

    public enum CameraPosition {
        TOWARDS_RINGS, // When facing backwards in starting position
        TOWARDS_GOAL // When forwards, lining up to shoot
    }

    public enum RingLifterPosition {
        UP,
        DOWN
    }

    public enum GoalLifterPosition {
        UP,
        MIDDLE,
        DOWN
    }

    public BotAppendages(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        cameraTurner = hardwareMap.get(Servo.class, "cameraTurner");

        shooterTilter = hardwareMap.get(Servo.class, "shooterTilter");
        shooterArm = hardwareMap.get(Servo.class, "shooterArm");
        shooterWheel = hardwareMap.get(DcMotorEx.class, "shooterWheel");
        shooterWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ringDetector = hardwareMap.get(ColorSensor.class, "ringDetector");
        ringLifter = hardwareMap.get(DcMotorEx.class, "ringRaiser");
        ringLifter.setDirection(DcMotorEx.Direction.REVERSE);
        ringLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ringLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ringLifter.setPositionPIDFCoefficients(1);

        largeIntakeRoller = hardwareMap.get(DcMotor.class, "largeIntakeRoller");
        smallIntakeRoller = hardwareMap.get(CRServo.class, "smallIntakeRoller");
        sideIntakeWheel = hardwareMap.get(CRServo.class, "sideIntakeWheel");
        sideIntakeWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftWingOpener = hardwareMap.get(Servo.class, "leftWingOpener");
        rightWingOpener = hardwareMap.get(Servo.class, "rightWingOpener");
        leftWingExtender = hardwareMap.get(Servo.class, "leftWingExtender");
        rightWingExtender = hardwareMap.get(Servo.class, "rightWingExtender");
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

    public void setCameraPosition(CameraPosition cameraPosition) {
        switch (cameraPosition) {
            case TOWARDS_RINGS:
                cameraTurner.setPosition(CAMERA_ANGLE_TOWARDS_RINGS);
                break;
            case TOWARDS_GOAL:
                cameraTurner.setPosition(CAMERA_ANGLE_TOWARDS_GOAL);
                break;
        }
    }

    public void setShooterTilterAngle(double angle) {
        double tilterAngle = MapUtil.map(angle, -1.0, 1.0, BotAppendages.MIN_SHOOTER_TILTER_ANGLE, BotAppendages.MAX_SHOOTER_TILTER_ANGLE);

        shooterTilter.setPosition(tilterAngle);
        //shooterTilter.setPosition(0); // For testing
    }

    public void enableShooterWheel(boolean enabled) {
        if (enabled) {
            shooterWheel.setVelocity(RING_SHOOTER_WHEEL_SPEED);
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

    public void setRingLifterPosition(RingLifterPosition position) {
        double lifterPosition;
        if (position == RingLifterPosition.UP) {
            lifterPosition = UP_RING_LIFTER_POSITION;
        } else {
            lifterPosition = DOWN_RING_LIFTER_POSITION;
        }

        ringLifter.setTargetPosition(EncoderUtil.inchesToTicks(EncoderUtil.Motor.REV_CORE_HEX, lifterPosition));
        ringLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ringLifter.setPower(RING_LIFTER_SPEED);
    }

    public void enableIntake(boolean enabled) {
        intakeEnabled = enabled;

        runIntake();
    }

    public void setIntakeDirection(Direction direction) {
        intakeDirection = direction;

        runIntake();
    }

    public void setInitialLeftWingPos(boolean extended) {
        leftWingLastExtended = extended;
    }

    public void extendLeftWing(boolean extend) {
        if (leftWingLastExtended == extend) return;
        leftWingLastExtended = extend;

        if (leftExtendThread != null) leftExtendThread.interrupt();

        Runnable extendTask = () -> {
            try {
                if (extend) {
                    leftWingOpener.setPosition(SUPER_OPEN_LEFT_WING_ANGLE);
                    Thread.sleep(OPEN_WING_DELAY);
                    leftWingExtender.setPosition(EXTENDED_LEFT_WING_ANGLE);
                    Thread.sleep(EXTEND_WING_DELAY);
                    leftWingOpener.setPosition(OPEN_LEFT_WING_ANGLE);
                    leftWingFullyOpened = true;
                } else {
                    leftWingFullyOpened = false;
                    leftWingOpener.setPosition(SUPER_OPEN_LEFT_WING_ANGLE);
                    // Could use CLOSE_WING_DELAY for better performance, but could break
                    Thread.sleep(OPEN_WING_DELAY);
                    leftWingExtender.setPosition(RETRACTED_LEFT_WING_ANGLE);
                    Thread.sleep(EXTEND_WING_DELAY);
                    leftWingOpener.setPosition(CLOSED_LEFT_WING_ANGLE);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        };

        leftExtendThread = new Thread(extendTask);
        leftExtendThread.start();
    }

    public void sweepLeftWing(boolean sweep) {
        if (!leftWingFullyOpened) return;

        if (sweep) {
            leftWingOpener.setPosition(SWEEP_LEFT_WING_ANGLE);
        } else {
            leftWingOpener.setPosition(OPEN_LEFT_WING_ANGLE);
        }
    }

    public void setInitialRightWingPos(boolean extended) {
        rightWingLastExtended = extended;
    }

    public void extendRightWing(boolean extend) {
        if (rightWingLastExtended == extend) return;
        rightWingLastExtended = extend;

        if (rightExtendThread != null) rightExtendThread.interrupt();

        Runnable extendTask = () -> {
            try {
                if (extend) {
                    rightWingOpener.setPosition(SUPER_OPEN_RIGHT_WING_ANGLE);
                    Thread.sleep(OPEN_WING_DELAY);
                    rightWingExtender.setPosition(EXTENDED_RIGHT_WING_ANGLE);
                    Thread.sleep(EXTEND_WING_DELAY);
                    rightWingOpener.setPosition(OPEN_RIGHT_WING_ANGLE);
                    rightWingFullyOpened = true;
                } else {
                    rightWingFullyOpened = false;
                    rightWingOpener.setPosition(SUPER_OPEN_RIGHT_WING_ANGLE);
                    // Could use CLOSE_WING_DELAY for better performance, but could break
                    Thread.sleep(OPEN_WING_DELAY);
                    rightWingExtender.setPosition(RETRACTED_RIGHT_WING_ANGLE);
                    Thread.sleep(EXTEND_WING_DELAY);
                    rightWingOpener.setPosition(CLOSED_RIGHT_WING_ANGLE);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        };

        rightExtendThread = new Thread(extendTask);
        rightExtendThread.start();
    }

    public void sweepRightWing(boolean sweep) {
        if (!rightWingFullyOpened) return;

        if (sweep) {
            rightWingOpener.setPosition(SWEEP_RIGHT_WING_ANGLE);
        } else {
            rightWingOpener.setPosition(OPEN_RIGHT_WING_ANGLE);
        }
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
                    Thread.sleep(700);
                    goalGrabber.setPosition(OPEN_GOAL_GRABBER_ANGLE);
                } else {
                    goalGrabber.setPosition(CLOSED_GOAL_GRABBER_ANGLE);
                    Thread.sleep(1000);
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
        double largeIntakeSpeed = 0;
        double smallIntakeSpeed = 0;
        double sideIntakeSpeed = 0;

        if (intakeEnabled) {
            double direction = intakeDirection == Direction.FORWARD ? 1 : -1;

            largeIntakeSpeed = BotAppendages.LARGE_INTAKE_ROLLER_SPEED * direction;
            smallIntakeSpeed = BotAppendages.SMALL_INTAKE_ROLLER_SPEED * direction;
            sideIntakeSpeed = BotAppendages.SIDE_INTAKE_ROLLER_SPEED * direction;
        }

        largeIntakeRoller.setPower(largeIntakeSpeed);
        smallIntakeRoller.setPower(smallIntakeSpeed);
        sideIntakeWheel.setPower(sideIntakeSpeed);
    }
}
