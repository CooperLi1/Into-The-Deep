package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class PinpointFusedLocalizer implements Localizer {
    GoBildaPinpointDriver pinpoint;

    public PinpointFusedLocalizer() {
        pinpoint = Globals.hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void init(Pose2d startPose) {
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-149.4375, 106.0);
//        pinpoint.setOffsets(106.0, -149.4375);

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(startPose);
    }

    @Override
    public void update() {
        pinpoint.update();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return pinpoint.getPosition();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        pinpoint.setPosition(pose);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return pinpoint.getVelocity();
    }
}
