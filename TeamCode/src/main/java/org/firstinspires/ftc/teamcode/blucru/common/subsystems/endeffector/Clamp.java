package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

@Config
public class Clamp extends BluServo implements Subsystem {
    public static double HORIZ_POS = 0.77;

    public Clamp() {
        super("clamp");
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

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
