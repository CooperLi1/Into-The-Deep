package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;

public class PivotMotor extends BluMotorWithEncoder {
    static final double TICKS_PER_RAD = 0.5;

    public PivotMotor() {
        super("pivot motor", Direction.FORWARD);
    }

    public double getAngle() {
        return getCurrentPosition() / TICKS_PER_RAD;
    }

    public double getAngleVel() {
        return getVelocity() / TICKS_PER_RAD;
    }
}
