package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Pivot;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Pivot PID Tuner", group = "test")
public class PivotPIDTuner extends BluLinearOpMode {
    public static double kP = 0.0, kI = 0.0, kD = 0.0, targetAngle = 0.0,
            MAX_VELO = 1.0, MAX_ACCEL = 0.5;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addPivot();
    }

    @Override
    public void periodic() {
        pivot.updatePID(kP, kI, kD);
        pivot.updateConstraints(MAX_VELO, MAX_ACCEL);

        if(!gamepad1.a && !gamepad1.b) {
            pivot.idle();
        } else if(stickyG1.a) {
            pivot.pivotTo(targetAngle);
        } else if(stickyG1.b) {
            pivot.setMotionProfileTargetAngle(targetAngle);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("Target Angle", targetAngle);
    }
}
