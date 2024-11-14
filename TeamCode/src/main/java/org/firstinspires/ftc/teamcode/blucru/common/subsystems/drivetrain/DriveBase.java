package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DriveKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.FusedLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.PinpointLocalizer;

public class DriveBase implements BluSubsystem {
    FusedLocalizer localizer;

    BluMotor fl, fr, bl, br;
    BluMotor[] motors;

    public DriveBase() {
        localizer = new FusedLocalizer(new PinpointLocalizer());

        fl = new BluMotor("fl");
        fr = new BluMotor("fr");
        bl = new BluMotor("bl");
        br = new BluMotor("br");

        motors = new BluMotor[] {fl, fr, bl, br};
    }

    @Override
    public void init() {
        for (BluMotor motor : motors) {
            motor.init();
        }
    }

    @Override
    public void read() {
        for (BluMotor motor : motors) {
            motor.read();
        }
    }

    @Override
    public void write() {
        for (BluMotor motor : motors) {
            motor.write();
        }
    }

    public void drive(Pose2d pose) {
        double[] powers = DriveKinematics.getDrivePowers(pose);
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }
}
