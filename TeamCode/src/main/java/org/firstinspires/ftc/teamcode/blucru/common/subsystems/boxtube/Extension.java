package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

@Config
public class Extension implements Subsystem {
    public static double
            kP = 0.0, kI = 0.0, kD = 0.0, kFAngle = 0.0, tolerance = 0.0,
            MIN_INCHES = 0.0, MAX_INCHES = 20,
            MAX_EXTEND_POWER = 0.8, MAX_RETRACT_POWER = -0.8;

    enum State {
        IDLE,
        PID,
        RETRACTING
    }

    State state;
    ExtensionMotor extensionMotor;
    PIDController pidController;

    PivotMotor pivot; // reference to pivot motor for feedforward

    public Extension() {
        extensionMotor = new ExtensionMotor();

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        state = State.IDLE;

        pivot = null;
    }

    @Override
    public void init() {
        extensionMotor.init();

        state = State.IDLE;
        pidController.reset();
    }

    @Override
    public void read() {
        extensionMotor.read();

        switch(state) {
            case IDLE:
            case PID:
                break;
            case RETRACTING:
                break;
        }
    }

    @Override
    public void write() {
        switch(state) {
            case IDLE:
                break;
            case PID:
            case RETRACTING:
                setPowerFeedForward(pidController.calculate());
                break;
        }

        extensionMotor.write();
    }

    public void pidTo(double inches) {
        inches = Range.clip(inches, MIN_INCHES, MAX_INCHES);

        state = State.PID;
        pidController.reset();
        pidController.setSetPoint(inches);
    }

    public void retract() {
        state = State.RETRACTING;
        pidController.setSetPoint(0);
    }

    public double getCurrentPos() {
        return extensionMotor.getCurrentPosition();
    }

    public double getFeedForward(double pivotAngle) {
        return Math.sin(pivotAngle) * kFAngle;
    }

    public void setPowerFeedForward(double power) {
        double ff;

        if(pivot == null) ff = 0;
        else ff = getFeedForward(pivot.getAngle());

        extensionMotor.setPower(Range.clip(power + ff, MAX_RETRACT_POWER, MAX_EXTEND_POWER));
    }

    public void updatePID() {
        pidController.setPID(kP, kI, kD);
    }

    public void idle() {
        state = State.IDLE;
        extensionMotor.setPower(0);
    }

    public void usePivot(PivotMotor pivot) {
        this.pivot = pivot;
    }

    public ExtensionMotor getMotor() {
        return extensionMotor;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Extension State", state);
        extensionMotor.telemetry();
    }
}
