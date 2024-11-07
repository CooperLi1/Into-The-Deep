package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

public class Arm extends BluServo implements Subsystem {
    public static double
            PARALLEL_POS = 0.5,
            MAX_POS = 1.0, MIN_POS = 0.0,
            TICKS_PER_RAD = 1/4.45059;

    enum State{
        SERVO,
        IVK
    }

    State state;
    double globalAngle;

    public Arm() {
        super("arm");
        state = State.SERVO;
    }

    @Override
    public void init() {
        super.init();
        globalAngle = Math.PI/2;
    }

    @Override
    public void write() {
        if(state == State.IVK) {
            super.setPosition(getAngle(globalAngle));
        }

        super.write();
    }

    public void setGlobalAngle(double globalAngle) {
        state = State.IVK;
        this.globalAngle = globalAngle;
    }

    public double getAngle(double globalAngle) {
        double rawTicks = toTicks(globalAngle - Robot.getInstance().pivot.getAngle() + PARALLEL_POS);
        return Range.clip(rawTicks, MIN_POS, MAX_POS);
    }

    public void setPosition(double position) {
        state = State.SERVO;
        super.setPosition(position);
    }

    public void parallel() {
        setPosition(PARALLEL_POS);
    }

    public void retract() {
        setPosition(PARALLEL_POS + 0.0);
    }

    public void dropToGround() {
        setPosition(PARALLEL_POS + 0.0);
    }

    public void turnToBucket() {
        setPosition(PARALLEL_POS + 0.0);
    }

    private double toTicks(double rad) {
        return rad * TICKS_PER_RAD;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
