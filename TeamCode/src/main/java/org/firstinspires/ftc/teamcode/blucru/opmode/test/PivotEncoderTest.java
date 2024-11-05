package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.PivotMotor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@TeleOp(name = "Pivot Encoder Test", group = "test")
public class PivotEncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.hwMap = hardwareMap;
        Globals.tele = telemetry;
        PivotMotor motor = new PivotMotor();
        motor.init();

        waitForStart();

        while(opModeIsActive()) {
            motor.read();

            motor.setPower(-gamepad1.right_stick_y);

            motor.write();
            telemetry.addData("angle", motor.getAngle());
            telemetry.addData("angle vel", motor.getAngleVel());
            telemetry.update();
        }
    }
}
