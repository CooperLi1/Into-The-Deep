package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

public class Wheel extends BluCRServo implements Subsystem {
    public Wheel() {
        super("wheel");
        setDirection(Direction.FORWARD);
    }

    public void intake() {
        super.setPower(1);
    }

    public void reverse() {
        super.setPower(-1);
    }

    public void stop() {
        super.setPower(0);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
