package org.firstinspires.ftc.teamcode.blucru.common.util;

// starting side for the robot in auto
public enum Side {
    RIGHT_SPECIMEN,
    LEFT_BASKETS;

    // flip the starting side
    public Side flip() {
        if(this == RIGHT_SPECIMEN) {
            return LEFT_BASKETS;
        } else {
            return RIGHT_SPECIMEN;
        }
    }
}
