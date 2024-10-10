package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@Config
@TeleOp(name = "Blu motor test", group = "test")
public class BluMotorTest extends LinearOpMode {
    public static String name = "intake motor";
    public static boolean reversed = true, useEncoder = true, brake = true;
    BluMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.hwMap = hardwareMap;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Globals.tele = telemetry;
        BluMotorBuilder builder = new BluMotorBuilder("intake motor");
        if(reversed) builder.reverse();
        if(useEncoder) builder.useEncoder();
        if(brake) builder.brake();
        motor = builder.build();

        waitForStart();

        while (opModeIsActive()) {
            motor.read();
            if(Math.abs(gamepad1.right_stick_y) > 0.02) {
                motor.setPower(-gamepad1.right_stick_y);
            } else motor.setPower(0);

            motor.write();
            motor.telemetry();
            telemetry.update();
        }
    }
}
