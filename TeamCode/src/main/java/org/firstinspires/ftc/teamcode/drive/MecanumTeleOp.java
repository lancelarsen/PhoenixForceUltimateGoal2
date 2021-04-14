package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.appendages.AppendagesAutonomous;
import org.firstinspires.ftc.teamcode.appendages.AppendagesTeleOp;
import org.firstinspires.ftc.teamcode.appendages.BotAppendages;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldPositions;
import org.firstinspires.ftc.teamcode.opmodes.teleop.util.GamepadUtils;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.sleep;

public class MecanumTeleOp extends BotMecanumDrive {
    private LinearOpMode opMode;

    private GamepadUtils driverGamepad;

    private double speedMultiplier = FAST_SPEED_MULTIPLIER;
    private boolean turningEnabled = false;

    public MecanumTeleOp(LinearOpMode opMode) {
        super(opMode.hardwareMap);
        this.opMode = opMode;

        driverGamepad = new GamepadUtils(opMode.gamepad1);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void enableTurning(boolean turningEnabled) {
        this.turningEnabled = turningEnabled;
    }

    public void arcadeDrive() {
        if (opMode.gamepad1.y) {
            speedMultiplier = FAST_SPEED_MULTIPLIER;
        } else if (opMode.gamepad1.b) {
            speedMultiplier = SLOW_SPEED_MULTIPLIER;
        } else if (opMode.gamepad1.x) {
            speedMultiplier = SUPER_SLOW_SPEED_MULTIPLIER;
        }

        Pose2d drivePower = new Pose2d();
        if (driverGamepad.isDpadActive()) {
            Gamepad gamepad = opMode.gamepad1;

            if (gamepad.dpad_up) {
                drivePower = new Pose2d(speedMultiplier, 0, 0);
            } else if (gamepad.dpad_down) {
                drivePower = new Pose2d(-speedMultiplier, 0, 0);
            }

            if (gamepad.dpad_left) {
                drivePower = new Pose2d(0, speedMultiplier, 0);
            } else if (gamepad.dpad_right) {
                drivePower = new Pose2d(0, -speedMultiplier, 0);
            }
        } else {
            drivePower = new Pose2d(
                    -opMode.gamepad1.left_stick_y * speedMultiplier,
                    -opMode.gamepad1.left_stick_x * speedMultiplier,
                    turningEnabled ? -opMode.gamepad1.right_stick_x * speedMultiplier : 0
                    );
        }

        setWeightedDrivePower(drivePower);
        update();
    }
}
