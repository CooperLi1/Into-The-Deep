package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluServoTest extends LinearOpMode {
    public static double position = 0.5;
    public static String name = "wrist";
    public static boolean reversed = false;
    BluServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.hwMap = hardwareMap;
        servo = new BluServo(name, reversed);

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                servo.setPosition(position);
            } else {
                servo.disable();
            }

            telemetry.addData("name", name);
            telemetry.addData("position", servo.getPosition());
            telemetry.update();
        }
    }
}
