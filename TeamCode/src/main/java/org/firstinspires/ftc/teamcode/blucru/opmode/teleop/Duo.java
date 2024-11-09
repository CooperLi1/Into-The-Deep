package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

import java.util.HashMap;

public class Duo extends BluLinearOpMode {
    enum State {
        RETRACTED,
        EXTENDED_OVER_INTAKE,
        INTAKING,
        RETRACTING_FROM_INTAKE,
        LIFTING_TO_BASKET,
        AT_BASKET,
        SCORING_BASKET,
        LIFTING_TO_SPECIMEN,
        AT_SPECIMEN,
        DUNKING_SPECIMEN,
        RETRACTING_FROM_SCORING
    }

    HashMap<State, Double> drivePowers;

    State state;
    StateMachine sm;

    @Override
    public void initialize() {
        setDrivePowers();

        addDrivetrain();
        addExtension();
        addPivot();
        addArm();
        addClamp();
        addWheel();
        addWrist();

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .transition(() -> gamepad2.left_bumper, State.EXTENDED_OVER_INTAKE, () -> {
                    extension.pidTo(7);
                    clamp.release();
                    arm.preIntake();
                })
                .transition(() -> gamepad2.right_bumper, State.EXTENDED_OVER_INTAKE, () -> {
                    extension.pidTo(15);
                    clamp.release();
                    arm.preIntake();
                })
                .transition(() -> gamepad2.dpad_right && !gamepad2.x, State.LIFTING_TO_BASKET, () -> {
                    pivot.pidTo(1.6);
                    wrist.uprightBackward();
                })
                .transition(() -> gamepad2.dpad_down && !gamepad2.x, State.LIFTING_TO_BASKET, () -> {
                    pivot.pidTo(1.6);
                    wrist.uprightBackward();
                })
                .transition(() -> gamepad2.dpad_left && !gamepad2.x, State.LIFTING_TO_SPECIMEN, () -> {
                    pivot.pidTo(1.6);
                    wrist.horizontal();
                })
                .transition(() -> gamepad2.dpad_up && !gamepad2.x, State.LIFTING_TO_SPECIMEN, () -> {
                    pivot.pidTo(1.6);
                    wrist.horizontal();
                })

                .state(State.EXTENDED_OVER_INTAKE)
                .transition(() -> -gamepad2.left_stick_y > 0.2, State.INTAKING, () -> {
                    arm.dropToGround();
                    wheel.intake();
                })
                .loop(() -> {
                    if(-gamepad2.left_trigger < -0.2) {
                        wheel.reverse();
                    } else {
                        wheel.stop();
                    }
                })

                .state(State.INTAKING)
                .transition(() -> gamepad2.a, State.RETRACTING_FROM_INTAKE, () -> {
                    arm.preIntake();
                    clamp.grab();
                    wheel.stop();
                })
                .transition(() -> -gamepad2.left_stick_y < 0.2, State.EXTENDED_OVER_INTAKE, () -> {
                    arm.preIntake();
                    wheel.stop();
                })

                .state(State.RETRACTING_FROM_INTAKE)
                .transitionTimed(0.3, State.RETRACTED, () -> {
                    extension.retract();
                    arm.retract();
                })
                .build();
    }

    @Override
    public void periodic() {
        updateDrivePower();
        dt.teleOpDrive(gamepad1); // driving
    }

    public void setDrivePowers() {
        drivePowers = new HashMap<State, Double>() {{
            put(State.RETRACTED, 0.9);
            put(State.EXTENDED_OVER_INTAKE, 0.7);
            put(State.INTAKING, 0.6);
            put(State.RETRACTING_FROM_INTAKE, 0.8);
            put(State.LIFTING_TO_BASKET, 0.6);
            put(State.AT_BASKET, 0.4);
            put(State.SCORING_BASKET, 0.3);
            put(State.LIFTING_TO_SPECIMEN, 0.6);
            put(State.AT_SPECIMEN, 0.4);
            put(State.DUNKING_SPECIMEN, 0.4);
            put(State.RETRACTING_FROM_SCORING, 0.7);
        }};
    }

    public void updateDrivePower() {
        try {
            dt.drivePower = drivePowers.get(state);
        } catch(NullPointerException e) {
            dt.drivePower = 0.5;
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", state);
    }
}
