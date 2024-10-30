package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

@Config
public class Pivot implements Subsystem {
    public static double
            kP = 0.0, kI = 0.0, kD = 0.0, tolerance = 0.0,
            kFF_ANGLE = 0.0, kFF_EXTENSION = 0.0,
            MIN_RAD = 0.0, MAX_RAD = 0.0,
            MAX_UP_POWER = 0.0, MAX_DOWN_POWER = 0.0;

    enum State {
        IDLE,
        PID,
        RETRACTING
    }

    State state;
    PIDController pidController;
    PivotMotor pivotMotor;
    LimitSwitch resetLimitSwitch;
    double feedForward;

    public Pivot() {
        pivotMotor = new PivotMotor();
        resetLimitSwitch = new LimitSwitch("reset switch");

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        state = State.IDLE;
    }

    @Override
    public void init() {
        pivotMotor.init();

        state = State.IDLE;
    }

    @Override
    public void read() {
        pivotMotor.read();

        switch(state) {
            case IDLE:
            case PID:
                break;
            case RETRACTING:
                if(resetLimitSwitch.isPressed()) {
                    state = State.IDLE;
                    pivotMotor.setPower(0);
                    pivotMotor.setCurrentPosition(0);
                    pidController.setSetPoint(0);
                    pidController.reset();
                }
                break;
        }
    }

    @Override
    public void write() {
        switch (state) {
            case IDLE:
                break;
            case PID:
            case RETRACTING:
                double power = Range.clip(pidController.calculate(pivotMotor.getCurrentPosition()), -MAX_DOWN_POWER, MAX_UP_POWER);
                pivotMotor.setPower(power);
                break;
        }

        pivotMotor.write();
    }

    public void pivotTo(double angle) {
        state = State.PID;
        pidController.setSetPoint(angle);
    }

    public void retract() {
        state = State.RETRACTING;
        pidController.setSetPoint(-0.1);
    }

    public double getCurrentPos() {
        return pivotMotor.getCurrentPosition();
    }

    public void setFeedForward(double extensionInches) {
        feedForward = kFF_ANGLE * (pivotMotor.getCurrentPosition() + kFF_EXTENSION * extensionInches);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        pivotMotor.telemetry();
        resetLimitSwitch.telemetry();
    }
}
