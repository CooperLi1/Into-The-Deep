package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

@Config
public class Wrist extends BluServo {
    public static double VERTICAL_POS = 0.5;

    public Wrist() {
        super("wrist");
    }

    public void uprightForward() {
        setPosition(VERTICAL_POS);
    }

    public void horizontal() {
        setPosition(VERTICAL_POS);
    }

    public void uprightBackward() {
        setPosition(VERTICAL_POS);
    }
}
