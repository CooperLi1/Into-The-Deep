package org.firstinspires.ftc.teamcode.common.util;

// starting side for the robot in auto
public enum Side {
    RIGHT,
    LEFT;


    // flip the starting side
    public Side flip() {
        if(this == RIGHT) {
            return LEFT;
        } else {
            return RIGHT;
        }
    }
}
