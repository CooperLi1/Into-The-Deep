package org.firstinspires.ftc.teamcode.blucru.common.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluMotor extends DcMotorImpl implements BluHardwareDevice {
    public static double RING_BUFFER_MILLIS = 8;

    String name;
    double power = 0, lastPower = 0;
    double encoderTicks = 0, vel = 0;
    double offsetTicks = 0;
    boolean useEncoder = false;
    double lastVelTime, lastPos;

    public BluMotor(DcMotor motor, String name, Direction direction, boolean useEncoder, ZeroPowerBehavior zeroPowerBehavior) {
        super(motor.getController(), motor.getPortNumber(), direction);
        this.name = name;
        this.useEncoder = useEncoder;
        super.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPower(double power) {
        // clip power to -1 to 1
        this.power = Range.clip(power, -1, 1);
    }

    public void init() {
        super.setPower(0);
        resetEncoder();
        offsetTicks = 0;
        lastVelTime = System.currentTimeMillis();
        lastPos = encoderTicks;
    }

    public void read() {
        // only update if encoder is being used
        if(useEncoder) {
            encoderTicks = super.getCurrentPosition();
            if(System.currentTimeMillis() - lastVelTime > RING_BUFFER_MILLIS) {
                double deltaPos = encoderTicks - lastPos;
                double deltaTime = System.currentTimeMillis() - lastVelTime;
                vel = deltaPos / (deltaTime / 1000.0);
                lastPos = encoderTicks;
                lastVelTime = System.currentTimeMillis();
            }
        }
    }

    public void write() {
        // only update if power has changed
        if(Math.abs(power - lastPower) > 0.005)
            super.setPower(power);

        lastPower = power;
    }

    public void resetEncoder() {
        setMode(RunMode.STOP_AND_RESET_ENCODER);
        setMode(RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setCurrentPosition(double pos) {
        offsetTicks = pos - encoderTicks;
    }

    public int getCurrentPosition() {
        return (int) (encoderTicks + offsetTicks);
    }

    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addData(name + " power", power);
        if(useEncoder) {
            tele.addLine(name + " pos: " + encoderTicks);
            tele.addLine(name + " vel: " + vel);
        }
    }
}
