package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class FieldPositions {

    //--- Starting positions
    public final static Pose2d S0 = new Pose2d(0, 0, SetFacing(Facing.GOAL));           //---X center of arena
    public final static Pose2d S1 = new Pose2d(-63.5, 52, SetFacing(Facing.GOAL));      //---X blue outside
    public final static Pose2d S2 = new Pose2d(-63.5, 11, SetFacing(Facing.GOAL));      //---X blue inside
    public final static Pose2d S3 = new Pose2d(-63.5, -16, SetFacing(Facing.GOAL));     //---X red inside
    public final static Pose2d S4 = new Pose2d(-63.5, -60, SetFacing(Facing.GOAL));     //---X red outside
    public final static Pose2d S5 = new Pose2d(-63.5, 0, SetFacing(Facing.GOAL));       //---X against wall, center

    //--- Wobble goal positions
    public final static Pose2d S1W = new Pose2d(-60, 53, SetFacing(Facing.GOAL));       //---X 2 inches away from wall, left blue
    public final static Pose2d S2W = new Pose2d(-60, 11, SetFacing(Facing.GOAL));       //---X 2 inches away from wall, right blue
    public final static Pose2d S3W = new Pose2d(-60, -16, SetFacing(Facing.GOAL));      //---X 2 inches away from wall, left red
    public final static Pose2d S4W = new Pose2d(-60, -60, SetFacing(Facing.GOAL));      //---X 2 inches away from wall, right red

    //--- Ring pickup positions
    public final static Pose2d R1 = new Pose2d(0, 0, SetFacing(Facing.GOAL)); //---
    public final static Pose2d R2 = new Pose2d(0, 0, SetFacing(Facing.GOAL)); //---
    public final static Pose2d R3 = new Pose2d(0, 0, SetFacing(Facing.BACK_ROTATE_RIGHT)); //---
    public final static Pose2d R4A = new Pose2d(-8, -37, Math.toRadians(-175)); //---
    public final static Pose2d R4B = new Pose2d(-20, -37, SetFacing(Facing.BACK_ROTATE_RIGHT)); //---
    public final static Pose2d R4C = new Pose2d(-27, -37, SetFacing(Facing.BACK_ROTATE_RIGHT)); //---

    //--- Transition positions
    public final static Pose2d X0 = new Pose2d(0, 0, SetFacing(Facing.GOAL));                   //--- center of the arena
    public final static Pose2d X1 = new Pose2d(0, 0, SetFacing(Facing.GOAL));                   //---
    public final static Pose2d X2 = new Pose2d(0, 0, SetFacing(Facing.GOAL));                   //---
    public final static Pose2d X3 = new Pose2d(45, 0, SetFacing(Facing.GOAL));                  //---X center of close to target zone
    public final static Pose2d X3R = new Pose2d(45, 0, SetFacing(Facing.BACK_ROTATE_RIGHT));    //---X center of close to target zone (reversed)

    public final static Pose2d X4 = new Pose2d(30, -15, SetFacing(Facing.BACK_ROTATE_RIGHT));   //---X center of close to target zone (reversed)
    public final static Pose2d X5R = new Pose2d(67, 0, SetFacing(Facing.BACK_ROTATE_RIGHT));     //---X far back center

    //--- White line positions
    public final static Pose2d L0 = new Pose2d(10, 0, SetFacing(Facing.GOAL));                  //---X center on white line
    public final static Pose2d L0R = new Pose2d(10, 0, SetFacing(Facing.BACK_ROTATE_RIGHT));    //--- center on white line (reversed)
    public final static Pose2d L1 = new Pose2d(10, 11, SetFacing(Facing.GOAL));                 //---XX blue side, park on line
    public final static Pose2d L1R = new Pose2d(10, 11, SetFacing(Facing.BACK_ROTATE_RIGHT));   //--- blue side, park on line (reversed)
    public final static Pose2d L2 = new Pose2d(10, -11, SetFacing(Facing.GOAL));                //--- red side, park on line
    public final static Pose2d L2R = new Pose2d(10, -11, SetFacing(Facing.BACK_ROTATE_RIGHT));  //---XX red side, park on line (reversed)
    public final static Pose2d L3 = new Pose2d(3, -61, SetFacing(Facing.RIGHT));                //---XX red side, park on line
    public final static Pose2d L4 = new Pose2d(1, 61, SetFacing(Facing.RIGHT));                 //---XX blue side, park on line

    //--- Wobble goal target positions
    public final static Pose2d W1 = new Pose2d(3, 60, SetFacing(Facing.GOAL));  //---
    public final static Pose2d W2 = new Pose2d(0, 0, SetFacing(Facing.GOAL));   //---
    public final static Pose2d W3 = new Pose2d(0, 0, SetFacing(Facing.GOAL));   //---
    public final static Pose2d W4 = new Pose2d(0, 0, SetFacing(Facing.GOAL));   //---
    public final static Pose2d W5 = new Pose2d(0, 0, SetFacing(Facing.GOAL));   //---
    public final static Pose2d W6 = new Pose2d(0, 0, SetFacing(Facing.GOAL));   //---

    //--- Wobble goal target positions (outside missions)
    public final static Pose2d W4O = new Pose2d(30, -61, SetFacing(Facing.LEFT));      //---X red A
    public final static Pose2d W5O = new Pose2d(52, -45, SetFacing(Facing.LEFT));       //---X red B
    public final static Pose2d W6O = new Pose2d(48, -59, SetFacing(Facing.RIGHT));      //---X red C
    public final static Pose2d W6O_2 = new Pose2d(44, -63, SetFacing(Facing.RIGHT));    //---X red C (2nd wobble)

    //--- Wobble goal target positions (inside missions)
    public final static Pose2d W4I = new Pose2d(27, -61, SetFacing(Facing.LEFT));       //---XX red A
    public final static Pose2d W5I = new Pose2d(35, -23, SetFacing(Facing.BACK_ROTATE_LEFT));       //---XX red B
    public final static Pose2d W6I = new Pose2d(58, -48, SetFacing(Facing.BACK_ROTATE_LEFT));      //---XX red C
//    public final static Pose2d W6I_2 = new Pose2d(44, -63, SetFacing(Facing.RIGHT));    //---X red C (2nd wobble)

    //--- Special path back from R OUT - Zone C to grab wobble goal
    public final static Pose2d S3G_1 = new Pose2d(63, -30, Math.toRadians(180));  //---
    public final static Pose2d S3G_2 = new Pose2d(-30, -30, SetFacing(Facing.BACK_ROTATE_LEFT));  //---
    public final static Pose2d S3G_3 = new Pose2d(-36, -30, Math.toRadians(72));     //--- red inside
    public final static Pose2d S3G_4 = new Pose2d(0, -63, SetFacing(Facing.LEFT));  //---X park on line

    //--- Wobble goals From top
    public final static Pose2d W1S = new Pose2d(3, 60, SetFacing(Facing.RIGHT));                //---
    public final static Pose2d W2S = new Pose2d(0, 0, SetFacing(Facing.RIGHT));                 //---
    public final static Pose2d W3S = new Pose2d(0, 0, SetFacing(Facing.RIGHT));                 //---
    public final static Pose2d W4S = new Pose2d(42, -35, SetFacing(Facing.BACK_ROTATE_RIGHT));  //---
    public final static Pose2d W5S = new Pose2d(50, -40, SetFacing(Facing.RIGHT));              //---
    public final static Pose2d W6S = new Pose2d(22, -40, SetFacing(Facing.BACK_ROTATE_RIGHT));  //---

    //--- Collect rings after shooting power shots positions
    public final static Pose2d C1A = new Pose2d(38, 11, SetFacing(Facing.GOAL));    //--- blue side collect rings from missed power shots
    public final static Pose2d C1B = new Pose2d(58, 11, SetFacing(Facing.GOAL));
    public final static Pose2d C2A = new Pose2d(38, -11, SetFacing(Facing.GOAL));   //--- red side collect rings from missed power shots
    public final static Pose2d C2B = new Pose2d(58, -11, SetFacing(Facing.GOAL));

    //--- Targeting positions
    public final static Pose2d T1 = new Pose2d(0, 0, SetFacing(Facing.GOAL));       //---
    public final static Pose2d T2 = new Pose2d(0, 0, SetFacing(Facing.GOAL));       //---
    public final static Pose2d T3 = new Pose2d(0, 0, SetFacing(Facing.GOAL));       //---
    public final static Pose2d T4 = new Pose2d(-4, -26, SetFacing(Facing.GOAL));    //---XX red center front line
    public final static Pose2d T5 = new Pose2d(-4, -61, Math.toRadians(23));        //---X red outside front line
    public final static Pose2d T6 = new Pose2d(0, 0, SetFacing(Facing.GOAL));       //---
    public final static Pose2d T7 = new Pose2d(-4, -16, Math.toRadians(-6));        //---X red inside front line

    //--- Power shot positions
    public final static Pose2d P1 = new Pose2d(0, 0, SetFacing(Facing.GOAL));       //---
    public final static double P1A[] = { -8, -8 };
    public final static Pose2d P2 = new Pose2d(0, 0, SetFacing(Facing.GOAL));       //---
    public final static double P2A[] = { -8, -8 };
    public final static Pose2d P3 = new Pose2d(0, 0, SetFacing(Facing.GOAL));       //---
    public final static double P3A[] = { 8, 8 };
    public final static Pose2d P4 = new Pose2d(-4, -10, Math.toRadians(-1));    //---X red inside power show
    public final static double P4A[] = { 9, 8 };
    public final static Pose2d P5 = new Pose2d(0, 0, SetFacing(Facing.GOAL));       //--- blue outside power show
    public final static double P5A[] = { 8, 8 };
    public final static Pose2d P6 = new Pose2d(-4, -61, Math.toRadians(33));        //---X red outside power shot
    public final static double P6A[] = { 5, 8 };

    private enum Facing
    {
        GOAL,
        LEFT,
        RIGHT,
        BACK_ROTATE_LEFT,
        BACK_ROTATE_RIGHT
    }

    public static double SetFacing(Facing facing)
    {
        double angle = 0;
        switch (facing)
        {
            case GOAL: angle = 0; break;
            case LEFT: angle = 90; break;
            case RIGHT: angle = -90; break;
            case BACK_ROTATE_LEFT: angle = -180; break;
            case BACK_ROTATE_RIGHT: angle = 180; break;
        }
        return Math.toRadians(angle);
    }
}
