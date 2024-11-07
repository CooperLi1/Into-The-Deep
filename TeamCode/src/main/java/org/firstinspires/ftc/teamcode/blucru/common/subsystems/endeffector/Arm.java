package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

public class Arm extends BluServo implements Subsystem {
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

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
