package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;

public class ExtensionMotor extends BluMotorWithEncoder {
    // TODO: calculate value
    static final double TICKS_PER_INCH = 83.796; // 145.1 on the motor

    public ExtensionMotor() {
        super("extension", Direction.FORWARD);
    }

    public double getDistance() {
        return getCurrentPosition() / TICKS_PER_INCH;
    }

    public double getDistanceVel() {
        return getVelocity() / TICKS_PER_INCH;
    }
}
