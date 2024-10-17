package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

import java.util.ArrayList;

@Config
public class Intake implements Subsystem {
    enum State{
        INTAKING,
        REVERSING,
        EMPTY,
        FULL
    }

    State state;
    ArrayList<BluHardwareDevice> devices;
    BluCRServo wheel;
    LimitSwitch limitSwitch;
    Clamp clamp;

    public Intake () {
        state = State.EMPTY;
        wheel = new BluCRServo("wheel");
        limitSwitch = new LimitSwitch("intake limit switch");
        clamp = new Clamp();

        devices = new ArrayList<>();
        devices.add(wheel);
        devices.add(clamp);
    }

    @Override
    public void init() {
        for(BluHardwareDevice device : devices) {
            device.init();
        }

        state = State.EMPTY;
    }

    @Override
    public void read() {
        for (BluHardwareDevice device : devices) {
            device.read();
        }

        switch(state) {
            case INTAKING:
                if(limitSwitch.isPressed()) {
                    // stop intaking
                    // clamp down
                    state = State.FULL;
                }
                break;
        }
    }

    @Override
    public void write() {
        for(BluHardwareDevice device : devices) {
            device.write();
        }
    }

    public void startIntaking() {
        state = State.INTAKING;
        wheel.setPower(1);
        clamp.release();
    }

    public void stopIntaking() {
        state = State.EMPTY;
        wheel.setPower(0);
        clamp.grab();
    }

    public void spitOut() {
        state = State.REVERSING;
        wheel.setPower(-1);
        clamp.release();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake State", state);
        for(BluHardwareDevice device : devices) {
            device.telemetry();
        }
    }
}
