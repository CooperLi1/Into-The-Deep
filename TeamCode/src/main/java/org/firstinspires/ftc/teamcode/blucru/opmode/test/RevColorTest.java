package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Rev Color Test", group = "test")
public class RevColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "color");

        waitForStart();

        color.initialize();

        while(opModeIsActive()) {
            float[] hsv = new float[3];
            NormalizedRGBA rgb = color.getNormalizedColors();
//            Color.colorToHSV(color.getNormalizedColors().toColor(), hsv);
            Color.RGBToHSV((int) (rgb.red * 255), (int) (rgb.green * 255), (int) (rgb.blue * 255), hsv);
            double distance = color.getDistance(DistanceUnit.INCH);

            telemetry.addData("hue", hsv[0]);
            telemetry.addData("saturation", hsv[1]);
            telemetry.addData("value", hsv[2]);
            telemetry.addData("distance", distance);
            telemetry.update();
        }
    }
}
