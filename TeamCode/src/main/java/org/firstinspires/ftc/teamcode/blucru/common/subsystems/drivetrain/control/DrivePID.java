package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;

//@Config
public class DrivePID {
    public static double
            kPX = 0.18, kIX = 0, kDX = 0.029,
            kPY = 0.18, kIY = 0, kDY = 0.029,
            kPHeading = 1.9, kIHeading = 0, kDHeading = 0.06;

    PIDController xController, yController, headingController;


    public DrivePID() {
        xController = new PIDController(kPX, kIX, kDX);
        yController = new PIDController(kPY, kIY, kDY);
        headingController = new PIDController(kPHeading, kIHeading, kDHeading);

    }

    public Vector2d calculate(Vector2d currentPosition) {
        double xPower = xController.calculate(currentPosition.getX());
        double yPower = yController.calculate(currentPosition.getY());
        return new Vector2d(xPower, yPower);
    }

    public void setTargetPosition(Vector2d targetPosition) {
        xController.setSetPoint(targetPosition.getX());
        yController.setSetPoint(targetPosition.getY());
    }

    public void updatePID() {
        xController.setPID(kPX, kIX, kDX);
        yController.setPID(kPY, kIY, kDY);
        headingController.setPID(kPHeading, kIHeading, kDHeading);
    }
}
