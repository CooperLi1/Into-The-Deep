package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class PivotMotor extends BluMotorWithEncoder {
    static final double TICKS_PER_RAD = 680.435;

    public PivotMotor() {
        super("pivot", Direction.REVERSE);
    }

    public double getAngle() {
        return getCurrentPosition() / TICKS_PER_RAD;
    }

    public double getAngleVel() {
        return getVelocity() / TICKS_PER_RAD;
    }

    @Override
    public void telemetry() {
        Globals.tele.addData("Pivot power", super.getPower());
        Globals.tele.addData("Pivot Angle", getAngle());
        Globals.tele.addData("Pivot Ang. Vel", getAngleVel());
    }
}
