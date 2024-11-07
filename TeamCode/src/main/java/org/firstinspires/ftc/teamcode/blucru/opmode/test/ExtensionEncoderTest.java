package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.ExtensionMotor;

public class ExtensionEncoderTest extends LinearOpMode {
    ExtensionMotor extension;

    @Override
    public void runOpMode() throws InterruptedException {
        extension = new ExtensionMotor();
        extension.init();

        waitForStart();

        while (opModeIsActive()) {
            extension.read();

            telemetry.addData("distance", extension.getDistance());
            telemetry.addData("distance vel", extension.getDistanceVel());
            telemetry.update();
        }
    }
}
