package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

public class BoxTube implements Subsystem {
    Extension extension;
    Pivot pivot;

    @Override
    public void init() {
        extension = new Extension();
        pivot = new Pivot();
    }

    @Override
    public void read() {
        extension.read();
        pivot.read();
    }

    @Override
    public void write() {
        extension.write();
        pivot.write();
    }

    public void updateFeedForward() {
        extension.updateFeedForward(pivot.getAngle());
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        extension.telemetry(telemetry);
        pivot.telemetry(telemetry);
    }
}
