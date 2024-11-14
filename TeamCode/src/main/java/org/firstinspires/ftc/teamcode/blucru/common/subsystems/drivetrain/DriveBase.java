package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        localizer = new FusedLocalizer();

        fl = new BluMotor("fl", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        fr = new BluMotor("fr", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        bl = new BluMotor("bl", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        br = new BluMotor("br", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

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
        localizer.update();

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

    public void driveFieldCentric(Pose2d fieldDrivePose) {
        Pose2d robotDrivePose = new Pose2d(fieldDrivePose.vec().rotated(-localizer.getHeading()), fieldDrivePose.getHeading());
        drive(robotDrivePose);
    }

    public Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose) {
        localizer.setPoseEstimate(pose);
    }

    public void setHeading(double heading) {
        localizer.setHeading(heading);
    }

    public double getHeading() {
        return localizer.getHeading();
    }

    public Pose2d getPoseVelocity() {
        return localizer.getPoseVelocity();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        localizer.telemetry();
    }
}
