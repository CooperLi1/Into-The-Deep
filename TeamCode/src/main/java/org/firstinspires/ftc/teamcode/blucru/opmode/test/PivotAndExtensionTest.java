package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

public class PivotAndExtensionTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addPivot();
        addExtension();
        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());
    }

    @Override
    public void periodic() {

    }
}
