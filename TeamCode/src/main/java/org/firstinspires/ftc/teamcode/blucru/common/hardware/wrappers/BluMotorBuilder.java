package org.firstinspires.ftc.teamcode.blucru.common.hardware.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;


public class BluMotorBuilder {
    String name;
    boolean reversed, useEncoder, brake;

    public BluMotorBuilder(String name) {
        this.name = name;
        this.reversed = false;
        this.useEncoder = false;
        this.brake = false;
    }

    public BluMotorBuilder reverse() {
        this.reversed = true;
        return this;
    }

    public BluMotorBuilder useEncoder() {
        this.useEncoder = true;
        return this;
    }

    public BluMotorBuilder brake() {
        this.brake = true;
        return this;
    }

    public BluMotor build() {
        DcMotor motor = Globals.hwMap.get(DcMotor.class, name);
        Direction direction = reversed ? Direction.REVERSE : Direction.FORWARD;
        ZeroPowerBehavior zeroPowerBehavior = brake ? ZeroPowerBehavior.BRAKE : ZeroPowerBehavior.FLOAT;
        return new BluMotor(motor, name,direction, useEncoder, zeroPowerBehavior);
    }
}
