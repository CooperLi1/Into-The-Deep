package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public class Wrist extends BluServo {
    public static double PARALLEL_POS = 0.5;

    public Wrist() {
        super("intake wrist");
    }

    public void parallel() {
        setPosition(PARALLEL_POS);
    }

    public void retract() {
        setPosition(PARALLEL_POS);
    }

    public void dropToIntake() {
        setPosition(PARALLEL_POS);
    }

    public void dropToScoring() {
        setPosition(PARALLEL_POS);
    }
}
