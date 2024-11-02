package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

@Config
public class Pivot implements Subsystem {
    public static double
            kP = 4.5, kI = 0.0, kD = 0.3, tolerance = 0.0,
            kFF_COS = 0.14, kFF_EXTENSION = 0.0,
            MIN_RAD = 0.0, MAX_RAD = 2,
            MAX_UP_POWER = 0.8, MAX_DOWN_POWER = -0.7,
            MAX_VELO = 1.0, MAX_ACCEL = 0.5;

    enum State {
        IDLE,
        PID,
        MOTION_PROFILE,
        RETRACTING
    }

    State state;
    PDController pidController;
    MotionProfile profile;
    PivotMotor pivotMotor;
    LimitSwitch resetLimitSwitch;
    double feedForward;

    public Pivot() {
        pivotMotor = new PivotMotor();
//        resetLimitSwitch = new LimitSwitch("reset switch");

        profile = new MotionProfile(0,0,MAX_VELO, MAX_ACCEL);
        pidController = new PDController(kP, kI, kD);
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
            case MOTION_PROFILE:
                break;
            case RETRACTING:
//                if(resetLimitSwitch.isPressed()) {
//                    state = State.IDLE;
//                    pivotMotor.setPower(0);
//                    pivotMotor.setCurrentPosition(0);
//                    pidController.setSetPoint(0);
//                    pidController.reset();
//                }
                break;
        }
    }

    @Override
    public void write() {
        switch (state) {
            case IDLE:
                setPivotPower(0);
                break;
            case PID:
                double pidPower = pidController.calculate(pivotMotor.getAngle());
                updateFFNoExtension();
                setPivotPower(pidPower + feedForward);
                break;
            case MOTION_PROFILE:
            case RETRACTING:
                double profilePower = pidController.calculate(pivotMotor.getAngle(), profile.getInstantTargetPosition(), pivotMotor.getAngleVel(), profile.getInstantTargetVelocity());
                updateFFNoExtension();
                setPivotPower(profilePower + feedForward);
                break;
        }

        pivotMotor.write();
    }

    public void setMotionProfileTargetAngle(double targetAngle) {
        targetAngle = Range.clip(targetAngle, MIN_RAD, MAX_RAD);
        state = State.MOTION_PROFILE;
        profile = new MotionProfile(targetAngle, pivotMotor.getAngle(), MAX_VELO, MAX_ACCEL);
    }

    public void pivotTo(double angle) {
        state = State.PID;
        pidController.setSetPoint(angle);
    }

    public void retract() {
        state = State.RETRACTING;
        pidController.setSetPoint(-0.1);
    }

    public double getAngle() {
        return pivotMotor.getAngle();
    }

    public void updateFFNoExtension() {
        feedForward = kFF_COS * Math.cos(pivotMotor.getAngle());
    }

    public void updateFF(double extensionInches) {
        feedForward = Math.cos(pivotMotor.getAngle()) * (kFF_COS + kFF_EXTENSION * extensionInches);
    }

    public void setTargetAngle(double angleRad) {
        pidController.setSetPoint(Range.clip(angleRad, MIN_RAD, MAX_RAD));
    }

    public void updatePID(double kP, double kI, double kD) {
        pidController.setPID(kP, kI, kD);
    }

    public void updateConstraints(double MAX_VELO, double MAX_ACCEL) {
        Pivot.MAX_VELO = MAX_VELO;
        Pivot.MAX_ACCEL = MAX_ACCEL;
    }

    private void setPivotPower(double power){
        pivotMotor.setPower(Range.clip(power, MAX_DOWN_POWER, MAX_UP_POWER));
    }

    public void idle() {
        state = State.IDLE;
        pivotMotor.setPower(0);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Pivot State", state);
        profile.telemetry(telemetry);
        pivotMotor.telemetry();
//        resetLimitSwitch.telemetry();
    }
}
