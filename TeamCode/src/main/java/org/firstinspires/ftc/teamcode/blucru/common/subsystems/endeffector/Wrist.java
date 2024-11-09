package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

@Config
public class Wrist extends BluServo implements Subsystem {
    public static double HORIZONTAL_POS = 0.43;

    public Wrist() {
        super("wrist");
    }

    @Override
    public void init() {
        super.init();
        uprightForward();
    }

    public void uprightForward() {
        setPosition(HORIZONTAL_POS - 0.28);
    }

    public void horizontal() {
        setPosition(HORIZONTAL_POS);
    }

    public void uprightBackward() {
        setPosition(HORIZONTAL_POS + 0.28);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
