package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

@Config
public class Clamp extends BluServo {
    public static double HORIZ_POS = 0.77;

    public Clamp() {
        super("intake clamp");
    }

    public void init() {
        super.init();
        release();
    }

    public void grab() {
        setPosition(HORIZ_POS - 0.17);
    }

    public void release() {
        setPosition(HORIZ_POS - 0.47);
    }
}
