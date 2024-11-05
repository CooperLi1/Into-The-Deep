package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class BoxTubePivotCommand extends InstantCommand {
    public BoxTubePivotCommand(double angle) {
        super(() -> {
//            Robot.getInstance().boxTube.pivotTo(angle);
        });
    }
}
