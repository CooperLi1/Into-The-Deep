package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

@Config
public class Wrist extends BluServo {
    public static double VERTICAL_POS = 0.5;

    public Wrist() {
        super("wrist");
    }

    public void vertical() {
        setPosition(VERTICAL_POS);
    }

    public void horizontalCCW() {
        setPosition(VERTICAL_POS);
    }

    public void horizontalCW() {
        setPosition(VERTICAL_POS);
    }
}
