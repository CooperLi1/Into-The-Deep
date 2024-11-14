package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DrivePID;

public class NewDrivetrain extends DriveBase implements Subsystem {
    enum State {
        IDLE,
        PID
    }

    State state;
    DrivePID pid;

    public NewDrivetrain() {
        super();
        state = State.IDLE;
        pid = new DrivePID();
    }

    @Override
    public void write() {
        switch (state) {
            case PID:
                driveFieldCentric(pid.calculate(getPoseEstimate()));
                break;
            case IDLE:
                break;
        }

        super.write();
    }

    public void setTargetPose(Pose2d targetPose) {
        pid.setTargetPose(targetPose);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Drivetrain State", state);
    }
}
