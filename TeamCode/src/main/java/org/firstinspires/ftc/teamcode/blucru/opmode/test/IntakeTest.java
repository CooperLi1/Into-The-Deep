package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

public class IntakeTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addIntake();
    }

    @Override
    public void periodic() {
        if(stickyG1.left_bumper) {
            intake.startIntaking();
        }

        if(stickyG1.right_bumper) {
            intake.spitOut();
        }

        if(stickyG1.a) {
            intake.stopIntaking();
        }
    }
}
