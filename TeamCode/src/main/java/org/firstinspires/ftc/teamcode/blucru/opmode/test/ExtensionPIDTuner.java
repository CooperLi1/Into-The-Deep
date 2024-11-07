package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

public class ExtensionPIDTuner extends BluLinearOpMode {
    public static double targetInches;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addExtension();
    }

    @Override
    public void periodic() {
        extension.updatePID();

        if(!(gamepad1.right_trigger > 0.2)) {
            extension.idle();
        } else if(gamepad1.a) {
            extension.pidTo(targetInches);
        }
    }

    @Override
    public void telemetry() {

    }
}
