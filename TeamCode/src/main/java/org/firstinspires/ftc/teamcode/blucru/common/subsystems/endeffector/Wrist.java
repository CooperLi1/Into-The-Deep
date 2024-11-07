package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

@Config
public class Wrist extends BluServo implements Subsystem {
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

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
