package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Intake Test", group = "test")
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
