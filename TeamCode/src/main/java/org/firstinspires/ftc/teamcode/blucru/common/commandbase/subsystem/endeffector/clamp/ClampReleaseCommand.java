package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem.endeffector.clamp;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ClampReleaseCommand extends InstantCommand {
    public ClampReleaseCommand() {
        super(
                () -> Robot.getInstance().clamp.release()
        );
    }
}
