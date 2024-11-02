package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Pivot;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Pivot PID Tuner", group = "test")
public class PivotPIDTuner extends BluLinearOpMode {
    public static double targetAngle = 0.0;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addPivot();
    }

    @Override
    public void periodic() {
        pivot.updatePID();

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
