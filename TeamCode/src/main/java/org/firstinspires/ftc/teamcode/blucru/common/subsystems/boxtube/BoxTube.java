package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;

public class BoxTube implements Subsystem {
    Extension extension;
    Pivot pivot;

    @Override
    public void init() {

    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    public void updateFeedForward() {
        extension.setFeedForward(pivot.getCurrentPos());
    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }
}
