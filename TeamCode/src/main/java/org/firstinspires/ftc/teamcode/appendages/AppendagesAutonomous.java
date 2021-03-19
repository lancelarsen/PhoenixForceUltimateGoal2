package org.firstinspires.ftc.teamcode.appendages;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AppendagesAutonomous extends BotAppendages {
    private HardwareMap hardwareMap;

    public AppendagesAutonomous(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.hardwareMap = hardwareMap;

        // Do unsafe wing retract
        // (set all servos to closed/retracted position, can break servos if wings not in)
        setInitialLeftWingPos(false);
        setInitialRightWingPos(false);
        leftWingOpener.setPosition(CLOSED_LEFT_WING_ANGLE);
        rightWingOpener.setPosition(CLOSED_RIGHT_WING_ANGLE);
        leftWingExtender.setPosition(RETRACTED_LEFT_WING_ANGLE);
        rightWingExtender.setPosition(RETRACTED_RIGHT_WING_ANGLE);
    }

    public Thread shootRing() {
        Runnable shootTask = () -> {
            try {
                extendShooterArm(true);
                Thread.sleep(500);
            } catch (Exception exception) {

            } finally {
                extendShooterArm(false);
            }
        };

        Thread shootThread = new Thread(shootTask);
        shootThread.start();

        return shootThread;
    }
}
