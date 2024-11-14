package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

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

    @Override
    public void telemetry(Telemetry telemetry) {

    }
}
