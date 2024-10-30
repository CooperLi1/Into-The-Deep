package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

@Config
public class Extension implements Subsystem {
    public static double
            kP = 0.0, kI = 0.0, kD = 0.0, kF = 0.0, tolerance = 0.0,
            MIN_INCHES = 0.0, MAX_INCHES = 0.0,
            MAX_EXTEND_POWER = 0.0, MAX_RETRACT_POWER = 0.0;

    enum State {
        IDLE,
        PID,
        RETRACTING
    }

    State state;
    ExtensionMotor extensionMotor;
    LimitSwitch resetLimitSwitch;

    PIDController pidController;
    double feedForward;

    public Extension() {
        extensionMotor = new ExtensionMotor();
        resetLimitSwitch = new LimitSwitch("reset switch");

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        state = State.IDLE;
        feedForward = 0.0;
    }

    @Override
    public void init() {
        extensionMotor.init();

        state = State.IDLE;
        pidController.reset();
        feedForward = 0.0;
    }

    @Override
    public void read() {
        extensionMotor.read();

        switch(state) {
            case IDLE:
            case PID:
                break;
            case RETRACTING:
                if(resetLimitSwitch.isPressed()) {
                    state = State.IDLE;
                    extensionMotor.setPower(0);
                    extensionMotor.setCurrentPosition(0);
                    pidController.setSetPoint(0);
                    pidController.reset();
                    feedForward = 0;
                }
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
                double power = Range.clip(pidController.calculate() + feedForward, -MAX_RETRACT_POWER, MAX_EXTEND_POWER) ;
                extensionMotor.setPower(power);
                break;
        }

        extensionMotor.write();
    }

    public void extendTo(double inches) {
        state = State.PID;
        pidController.reset();
        pidController.setSetPoint(inches);
    }

    public void retract() {
        state = State.RETRACTING;
        pidController.setSetPoint(-1);
    }

    public double getCurrentPos() {
        return extensionMotor.getCurrentPosition();
    }

    public void setFeedForward(double pivotAngle) {
        feedForward = Math.sin(pivotAngle) * kF;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Extension State", state);
        telemetry.addData("Extension feed forward", feedForward);
        extensionMotor.telemetry();
        resetLimitSwitch.telemetry();
    }
}
