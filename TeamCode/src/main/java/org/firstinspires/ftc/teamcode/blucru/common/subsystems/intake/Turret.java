package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

@Config
public class Turret extends BluServo {
    public static double VERTICAL_POS = 0.5;

    public Turret () {
        super("intake turret");
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
