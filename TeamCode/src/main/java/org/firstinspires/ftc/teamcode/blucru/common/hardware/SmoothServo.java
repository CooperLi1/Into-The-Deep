package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.wrappers.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.wrappers.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class SmoothServo extends BluServo implements BluHardwareDevice {
    final static double defaultKPrev = 0.03;

    /*

    A kPrev of 0 will result in no smoothing,
    while a kPrev of 1 will result in most smoothing

    Lower kPrev = less smoothing
    Higher kPrev = more smoothing

    keep in mind, a smooth servo will continuously
    be updating position, which results in longer loop times

     */

    double kPrev; // weight of previous position in smoothing
    double finalPosition, currentPosition;

    public SmoothServo(String name, Direction direction) {
        super(name, direction);
        kPrev = defaultKPrev;
    }

    public SmoothServo(String name, boolean reversed) {
        super(name, reversed);
        kPrev = defaultKPrev;
    }

    public SmoothServo(String name, boolean reversed, double kPrev) {
        super(name, reversed);
        this.kPrev = kPrev;
    }

    public SmoothServo(String name) {
        super(name);
        kPrev = defaultKPrev;
    }

    public SmoothServo(String name, double kPrev) {
        super(name);
        this.kPrev = kPrev;
    }

    public SmoothServo(String name, Direction direction, double kPrev) {
        super(name, direction);
        this.kPrev = kPrev;
    }

    public void init() {
        super.init();
    }

    public void read() {
        super.read();
    }

    public void write() {
        super.setPosition(kPrev * super.getPosition() + (1 - kPrev) * finalPosition);
        super.write();
    }

    public void setPosition(double position) {
        finalPosition = position;
    }

    public void setKPrev(double kPrev) {
        this.kPrev = kPrev;
    }

    public void telemetry() {
        Globals.tele.addData(super.getName() + "target pos", finalPosition);
        super.telemetry();
    }
}