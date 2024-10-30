package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;

public class PDController extends PIDController {
    Vector2d k;
    public PDController(double kP, double kI, double kD) {
        super(kP, kI, kD);
        k = new Vector2d(kP, kD);
    }

    public double calculate(double currentPos, double targetPos, double currentVelocity, double targetVelocity) {
        Vector2d pv = new Vector2d(currentPos, currentVelocity);
        Vector2d sp = new Vector2d(targetPos, targetVelocity);
        Vector2d error = sp.minus(pv);

        return error.dot(k);
    }
}