package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

import java.util.ArrayList;

@Config
public class Intake implements Subsystem {
    enum State{
        INTAKING,
        REVERSING,
        RETRACTED_EMPTY,
        FULL
    }

    State state;
    ArrayList<BluHardwareDevice> devices;
    BluCRServo wheel;
//    LimitSwitch limitSwitch;
    Clamp clamp;
    Turret turret;
    Wrist wrist;

    public Intake () {
        state = State.RETRACTED_EMPTY;
        wheel = new BluCRServo("intake wheel");
//        limitSwitch = new LimitSwitch("intake limit switch");
        clamp = new Clamp();
        turret = new Turret();
        wrist = new Wrist();

        devices = new ArrayList<>();
        devices.add(wheel);
        devices.add(clamp);
        devices.add(turret);
        devices.add(wrist);
    }

    @Override
    public void init() {
        for(BluHardwareDevice device : devices) {
            device.init();
        }

        state = State.RETRACTED_EMPTY;
    }

    @Override
    public void read() {
        for (BluHardwareDevice device : devices) {
            device.read();
        }

        switch(state) {
            case INTAKING:
//                if(limitSwitch.isPressed()) {
//                    // stop intaking
//                    // clamp down
//                    state = State.FULL;
//                }
                break;
            case REVERSING:
            case RETRACTED_EMPTY:
                turret.vertical();
                wrist.retract();
                break;
            case FULL:
                break;
        }
    }

    @Override
    public void write() {
        for(BluHardwareDevice device : devices) {
            device.write();
        }
    }

    public void retract() {
        state = State.RETRACTED_EMPTY;
        turret.vertical();
        wrist.retract();
    }

    public void startIntaking() {
        state = State.INTAKING;
        wheel.setPower(1);
        clamp.release();
    }

    public void stopIntaking() {
        state = State.RETRACTED_EMPTY;
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
