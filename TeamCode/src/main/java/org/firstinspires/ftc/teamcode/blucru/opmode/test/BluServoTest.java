package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.wrappers.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@Config
@TeleOp(name = "Blu servo test", group = "test")
public class BluServoTest extends LinearOpMode {
    public static double position = 0.5;
    public static String name = "wrist";
    public static boolean reversed = false;
    BluServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.hwMap = hardwareMap;
        Globals.tele = telemetry;
        servo = new BluServo(name, reversed);

        waitForStart();
        while(opModeIsActive()) {
            servo.read();

            if(gamepad1.a) {
                servo.setPosition(position);
            } else if (gamepad1.b) {
                servo.rawSetPosition(position);
            } else {
                servo.disable();
            }

            servo.write();
            servo.telemetry();
            telemetry.update();
        }
    }
}
