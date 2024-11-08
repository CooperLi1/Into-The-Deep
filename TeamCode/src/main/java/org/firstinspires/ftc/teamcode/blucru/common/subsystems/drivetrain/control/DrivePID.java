package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;

//@Config
public class DrivePID {
    public static double kPX = 0.18, kIX = 0, kDX = 0.029;
    public static double kPY = 0.18, kIY = 0, kDY = 0.029;
    public static double kPHeading = 1.9, kIHeading = 0, kDHeading = 0.06;
    double tolerance;

    PIDController xController, yController, headingController;

    Vector2d targetPosition;

    public DrivePID(double kP, double kI, double kD, double tolerance) {
        this.tolerance = tolerance;

        xController = new PIDController(kPX, kIX, kDX);
        yController = new PIDController(kPY, kIY, kDY);
        headingController = new PIDController(kPHeading, kIHeading, kDHeading);

        targetPosition = new Vector2d(0, 0);
    }

    public Vector2d calculate(Vector2d currentPosition) {
        Vector2d error = targetPosition.minus(currentPosition);
        if(error.norm() < tolerance) {
            return new Vector2d(0,0);
        } else {
            double xPower = xController.calculate(currentPosition.getX(), targetPosition.getX());
            double yPower = yController.calculate(currentPosition.getY(), targetPosition.getY());
            return new Vector2d(xPower, yPower);
        }
    }

    public void setTargetPosition(Vector2d targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void setPID(double kP, double kI, double kD) {
        xController.setP(kP);
        xController.setI(kI);
        xController.setD(kD);
        yController.setP(kP);
        yController.setI(kI);
        yController.setD(kD);
    }

    public void setkP(double kP) {
        xController.setP(kP);
        yController.setP(kP);
    }

    public void setkI(double kI) {
        xController.setI(kI);
        yController.setI(kI);
    }

    public void setkD(double kD) {
        xController.setD(kD);
        yController.setD(kD);
    }
}
