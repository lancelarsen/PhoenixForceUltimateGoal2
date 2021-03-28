package org.firstinspires.ftc.teamcode.appendages;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.sleep;

public class AppendagesAutonomous extends BotAppendages {
    private final static long SHOOTER_ARM_EXTEND_DELAY = 350;
    private final static long SHOOTER_ARM_RETRACT_DELAY = 700;

    private OpMode opMode;

    public AppendagesAutonomous(LinearOpMode opMode) {
        super(opMode.hardwareMap);
        this.opMode = opMode;
    }

    public void shootRings() {
        shootRings(3);
    }

    public void shootRings(int num) {
        for(int i = 0; i < num; i++) {
            extendShooterArm(true);
            sleep(SHOOTER_ARM_EXTEND_DELAY);
            extendShooterArm(false);
            sleep(SHOOTER_ARM_RETRACT_DELAY);
        }
    }
}
