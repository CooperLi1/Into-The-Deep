package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class DriveKinematics {
    public static double AXIAL_DECELERATION = 0.5,
        LATERAL_DECELERATION = 0.5,
        HEADING_DECELERATION = 0.5;

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
        return new Pose2d();
    }
}
