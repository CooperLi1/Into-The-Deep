package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Drive Test", group = "test")
public class DriveTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addDrivetrain();
        enableFTCDashboard();

        dt.drivePower = 0.8;
    }

    @Override
    public void periodic() {
        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;

        if(gamepad1.right_stick_button) {
            dt.resetHeading(Math.PI/2);
            gamepad1.rumble(150);
        }

        dt.teleOpDrive(horz, vert, rotate);
        dt.ftcDashDrawCurrentPose();
    }

    @Override
    public void telemetry() {
        dt.testTelemetry(telemetry);
    }
}
