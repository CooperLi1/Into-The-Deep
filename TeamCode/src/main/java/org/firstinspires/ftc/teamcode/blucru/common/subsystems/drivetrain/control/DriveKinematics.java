package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;

@Config
public class DriveKinematics {
    public static double
        AXIAL_DECEL = 0.5, LATERAL_DECEL = 0.5, HEADING_DECEL = 8;

    public static double getHeadingTowardsPoint(Pose2d currentPose, Pose2d targetPose) {
        return Math.atan2(targetPose.getY() - currentPose.getY(),
                            targetPose.getX() - currentPose.getX());
    }

    public static double getHeadingVelocityTowardsPoint(Pose2d currentPose, Pose2d targetPose, Pose2d currentVelocity) {
        double xDelta = targetPose.getX() - currentPose.getX();
        double yDelta = targetPose.getY() - currentPose.getY();

        double xVel = currentVelocity.getX();
        double yVel = currentVelocity.getY();

        // derive atan(yDelta / xDelta) with respect to time
        return (xDelta * yVel - yDelta * xVel) / (xDelta * xDelta + yDelta * yDelta);
    }

    public static Pose2d getStopPose(Pose2d currentPose, Pose2d currentVelocity) {
        Pose2d robotVel = new Pose2d(currentVelocity.vec().rotated(-currentPose.getHeading()), currentVelocity.getHeading());

        // absolute value does the same as Math.signum because its squared, so the sign is preserved
        double robotDeltaX = robotVel.getX() * Math.abs(robotVel.getX()) / (2 * AXIAL_DECEL);
        double robotDeltaY = robotVel.getY() * Math.abs(robotVel.getY()) / (2 * LATERAL_DECEL);
        double robotDeltaHeading = robotVel.getHeading() * Math.abs(robotVel.getHeading()) / (2 * HEADING_DECEL);

        Pose2d robotDeltaPose = new Pose2d(robotDeltaX, robotDeltaY, robotDeltaHeading);

        Pose2d globalDeltaPose = new Pose2d(robotDeltaPose.vec().rotated(currentPose.getHeading()), robotDeltaPose.getHeading());

        return new Pose2d(currentPose.vec().plus(globalDeltaPose.vec()), Angle.norm(currentPose.getHeading() + globalDeltaPose.getHeading()));
    }
}
