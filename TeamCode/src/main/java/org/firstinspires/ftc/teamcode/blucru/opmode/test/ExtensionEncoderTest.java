package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.ExtensionMotor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@TeleOp(name = "Extension Encoder Test", group = "test")
public class ExtensionEncoderTest extends LinearOpMode {
    ExtensionMotor extension;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.hwMap = hardwareMap;
        Globals.tele = telemetry;

        extension = new ExtensionMotor();
        extension.init();

        waitForStart();

        while (opModeIsActive()) {
            extension.read();

            extension.setPower(-gamepad1.right_stick_y);

            extension.write();

            telemetry.addData("distance", extension.getDistance());
            telemetry.addData("distance vel", extension.getDistanceVel());
            telemetry.update();
        }
    }
}
