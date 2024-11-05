package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "raw drive test", group = "test")
public class RawDriveTest extends BluLinearOpMode {
    double vert, horz, rotate;
    @Override
    public void initialize() {
        addDrivetrain();
    }

    @Override
    public void periodic() {
        vert = -gamepad1.left_stick_y;
        horz = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        dt.driveScaled(horz ,vert, rotate);
    }

    @Override
    public void telemetry() {
        dt.testTelemetry(telemetry);
    }
}
