package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

@Config
public class Clamp extends BluServo {
    public static double OPEN_POS = 0.5, CLOSED_POS = 0.5;

    public Clamp() {
        super("intake clamp");
    }

    public void grab() {
        setPosition(CLOSED_POS);
    }

    public void release() {
        setPosition(OPEN_POS);
    }
}
