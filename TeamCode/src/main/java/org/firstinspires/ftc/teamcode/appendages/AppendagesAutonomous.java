package org.firstinspires.ftc.teamcode.appendages;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtils.sleep;

public class AppendagesAutonomous extends BotAppendages {
    private final static long GOAL_GRABBER_CLOSE_DELAY = 700;
    private final static long GOAL_GRABBER_OPEN_DELAY = 500;

    private OpMode opMode;

    public AppendagesAutonomous(LinearOpMode opMode) {
        super(opMode.hardwareMap);
        this.opMode = opMode;

        openGoalGrabber(true);
    }

    public void wobbleGoalGrab() {
        wobbleGoalGrab(GoalLifterPosition.MIDDLE);
    }

    public void wobbleGoalGrab(GoalLifterPosition lifterPosition) {
        if (lifterPosition == GoalLifterPosition.DOWN) return;

        openGoalGrabber(false);
        sleep(GOAL_GRABBER_CLOSE_DELAY);
        setGoalLifterPosition(lifterPosition);
    }

    public void wobbleGoalDrop() {
        setGoalLifterPosition(GoalLifterPosition.DOWN);
        while (goalLifter.isBusy()); // Wait for lifter to go down, better than using delay //TODO: Not working
        sleep(500); //TODO: temporary
        openGoalGrabber(true);
        sleep(GOAL_GRABBER_OPEN_DELAY);
    }
}
