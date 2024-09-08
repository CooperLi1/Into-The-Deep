package org.firstinspires.ftc.teamcode.opmode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "CRServo test", group = "hardware test")
public class CRServoTest extends LinearOpMode {
    public static String name = "intake roller";
    public static boolean reversed = false;
    CRServo test;
    @Override
    public void runOpMode() throws InterruptedException {
        test = hardwareMap.get(CRServo.class, name);
        waitForStart();
        while(opModeIsActive()) {
            updateName();
            updateDirection();

            if(Math.abs(gamepad1.left_stick_y) > 0.1) test.setPower(-gamepad1.left_stick_y);
            else test.setPower(0);


            telemetry.addData("name", name);
            telemetry.addData("power", test.getPower());
            telemetry.update();
        }
    }

    public void updateName() {
        try {
            test = hardwareMap.get(CRServoImplEx.class, name);
        } catch (Exception e) {
            telemetry.addLine("ERROR: servo " + name + " not found");
        }
    }

    public void updateDirection() {
        try {
            if(reversed) test.setDirection(CRServo.Direction.REVERSE);
            else test.setDirection(CRServo.Direction.FORWARD);
        } catch (Exception ignored) {}
    }
}
