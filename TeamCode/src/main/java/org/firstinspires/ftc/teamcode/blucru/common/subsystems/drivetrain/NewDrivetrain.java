package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DrivePID;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class NewDrivetrain extends SampleMecanumDrive implements BluSubsystem, Subsystem {
    enum State {
        IDLE,
        PID
    }

    State state;
    DrivePID pid;

    public NewDrivetrain(HardwareMap hwMap) {
        super(hwMap);
        state = State.IDLE;
        pid = new DrivePID();
    }

    @Override
    public void init() {

    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }
}
