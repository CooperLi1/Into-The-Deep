package org.firstinspires.ftc.teamcode.blucru.common.commandbase.system.endeffector;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem.endeffector.wrist.WristHorizontalCommand;

public class EndEffectorRetractCommand extends SequentialCommandGroup {
    public EndEffectorRetractCommand() {
        super(
                new ClampGrabCommand(),
                new ArmRetractCommand(),
                new WheelStopCommand(),
                new WaitCommand(300),
                new WristHorizontalCommand()
        );
    }
}
