package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public class Arm extends BluServo {
    public static double PARALLEL_POS = 0.5;

    public Arm() {
        super("arm");
    }

    public void parallel() {
        setPosition(PARALLEL_POS);
    }

    public void retract() {
        setPosition(PARALLEL_POS);
    }

    public void dropToGround() {
        setPosition(PARALLEL_POS);
    }

    public void turnToBucket() {
        setPosition(PARALLEL_POS);
    }
}
