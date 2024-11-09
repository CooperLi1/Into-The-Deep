package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem.endeffector.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ArmHorizontalCommand extends InstantCommand {
    public ArmHorizontalCommand() {
        super(
                () -> Robot.getInstance().arm.preIntake()
        );
    }
}
