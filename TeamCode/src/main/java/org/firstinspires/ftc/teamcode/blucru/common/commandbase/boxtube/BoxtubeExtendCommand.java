package org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class BoxtubeExtendCommand extends SequentialCommandGroup {
    public BoxtubeExtendCommand(double pivotAngle, double extensionDistance) {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new PivotCommand(pivotAngle),

                                // Wait time depends on distance pivot needs to rotate. Longer rotation requires longer wait time.
                                // here i do distance in radians times 150 to get the time in milliseconds, so at 1 radian it would be 150ms
                                new WaitCommand((long) (Robot.getInstance().pivot.getAngle() - pivotAngle * 150.0)),
                                new ExtensionCommand(extensionDistance)
                        ),
                        new SequentialCommandGroup(
                                new ExtensionCommand(extensionDistance),

                                // Wait time depends on distance extension needs to extend. Longer extension requires longer wait time.
                                // here i do distance in inches times 15, so at 15 inches it will wait 225 ms
                                new WaitCommand((long) (Robot.getInstance().extension.getDistance() - extensionDistance * 15.0)),
                                new PivotCommand(pivotAngle)
                        ),

                        () -> extensionDistance > Robot.getInstance().extension.getDistance()
                )
        );
    }
}
