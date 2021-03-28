package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Arrays;

public class MecanumAuto extends BotMecanumDrive {
    // In inches per second
    private static double SLOW_MAX_VEL = 8;
    private static double MEDIUM_MAX_VEL = 15;
    private static double FAST_MAX_VEL = 50;

    public enum Speed {
        SLOW,
        MEDIUM,
        FAST
    }

    public enum MovementMethod {
        LINE,
        SPLINE,
        SPLINE_CONSTANT_HEADING
    }

    private Pose2d position;

    public MecanumAuto(LinearOpMode opMode) {
        super(opMode.hardwareMap);
    }

    public void setCurrentPosition(Pose2d position) {
        this.position = position;
        setPoseEstimate(position);
    }

    public Builder builder(Speed speed) {
        return builder(speed, false);
    }

    public Builder builder(Speed speed, boolean reversed) {
        return new Builder(speed, reversed);
    }

    public class Builder {
        private ArrayList<TrajectoryPoint> points = new ArrayList<>();

        private Speed speed;
        private boolean reversed;

        private Builder(Speed speed, boolean reversed) {
            this.speed = speed;
            this.reversed = reversed;
        }

        public Builder line(Pose2d pose2d) {
            points.add(new TrajectoryPoint(pose2d, MovementMethod.LINE));

            return this;
        }

        public Builder spline(Pose2d pose2d, double splineAngle) {
            points.add(new TrajectoryPoint(pose2d, MovementMethod.SPLINE, splineAngle));

            return this;
        }

        public Builder splineConstantHeading(Pose2d pose2d, double splineAngle) {
            points.add(new TrajectoryPoint(pose2d, MovementMethod.SPLINE_CONSTANT_HEADING, splineAngle));

            return this;
        }

        public void follow() {
            TrajectoryBuilder builder = trajectoryBuilder(position, reversed);

            double maxVel = 0;
            switch (speed) {
                case SLOW:
                    maxVel = SLOW_MAX_VEL; break;
                case MEDIUM:
                    maxVel = MEDIUM_MAX_VEL; break;
                case FAST:
                    maxVel = FAST_MAX_VEL; break;
            }

            MinVelocityConstraint velConstraint = new MinVelocityConstraint(
                    Arrays.asList(
                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                            new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                    )
            );
            ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);

            for (TrajectoryPoint point : points) {
                switch (point.movementMethod) {
                    case LINE:
                        builder.lineToLinearHeading(point.pose2d, velConstraint, accelConstraint);
                        break;
                    case SPLINE:
                        builder.splineToLinearHeading(point.pose2d, Math.toRadians(point.splineAngle), velConstraint, accelConstraint);
                        break;
                    case SPLINE_CONSTANT_HEADING:
                        builder.splineToConstantHeading(point.pose2d.vec(), Math.toRadians(point.splineAngle), velConstraint, accelConstraint);
                        break;
                }
            }

            Trajectory trajectory = builder.build();
            followTrajectory(trajectory);
            position = trajectory.end();
        }

        private class TrajectoryPoint {
            Pose2d pose2d;
            MovementMethod movementMethod;
            double splineAngle;

            TrajectoryPoint(Pose2d pose2d, MovementMethod movementMethod) {
                this.pose2d = pose2d;
                this.movementMethod = movementMethod;
            }

            TrajectoryPoint(Pose2d pose2d, MovementMethod movementMethod, double splineAngle) {
                this.pose2d = pose2d;
                this.movementMethod = movementMethod;
                this.splineAngle = splineAngle;
            }
        }
    }
}
