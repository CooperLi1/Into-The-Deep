package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Arm IVK Test", group = "test")
public class ArmIVKTest extends BluLinearOpMode {
    public static double globalAngle = Math.PI/2;

    @Override
    public void initialize() {
        addPivot();
        addArm();
    }

    @Override
    public void periodic() {
        if(gamepad1.a) {
            arm.setGlobalAngle(globalAngle);
        } else {
            arm.disable();
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("global angle", globalAngle);
    }
}
