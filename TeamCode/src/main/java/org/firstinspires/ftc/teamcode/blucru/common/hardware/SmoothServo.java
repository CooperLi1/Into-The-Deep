package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.wrappers.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.wrappers.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

public class SmoothServo extends BluServo implements BluHardwareDevice {
    static final double defaultVmax = 0.5, defaultAmax = 0.5;

    MotionProfile motionProfile;
    double finalPosition, currentPosition;
    double vMax, aMax;

    public SmoothServo(String name, boolean reversed) {
        this(name, reversed, defaultVmax, defaultAmax);
    }

    public SmoothServo(String name, boolean reversed, double vMax, double aMax) {
        super(name, reversed);
        this.vMax = vMax;
        this.aMax = aMax;
    }

    public void init() {
        super.init();
    }

    public void read() {
        super.read();
    }

    public void write() {
        if(motionProfile != null) {
            currentPosition = motionProfile.getInstantTargetPosition();
            super.setPosition(currentPosition);
        }
    }

    public void setPosition(double position) {
        finalPosition = position;
        motionProfile = new MotionProfile(currentPosition, finalPosition, vMax, aMax).start();
    }

    public void setConstraints(double vMax, double aMax) {
        this.vMax = vMax;
        this.aMax = aMax;
    }

    public void telemetry() {
        addLine("Target Pos: " + finalPosition);
        addLine("Current Pos: " + currentPosition);
//        Globals.tele.addData(super.getName() + "target pos", finalPosition);
//        Globals.tele.addData(super.getName() + "current pos", currentPosition);
        super.telemetry();
    }
}