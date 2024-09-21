package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.SmoothServo;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@Config
@TeleOp(name = "Smooth servo test", group = "test")
public class SmoothServoTest extends LinearOpMode {
    public static String name = "wrist";
    public static boolean reversed = false;
    public static double kPrev = 0.1;
    public static double position = 0.5;

    SmoothServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.hwMap = hardwareMap;
        Globals.tele = telemetry;

        servo = new SmoothServo(name, reversed, kPrev);

        waitForStart();

        while(opModeIsActive()) {
            servo.read();

            if(gamepad1.a) {
                servo.setPosition(position);
            } else {
                servo.disable();
            }

            servo.write();
            servo.telemetry();
            telemetry.update();
        }
    }
}
