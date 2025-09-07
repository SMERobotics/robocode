package com.technodot.ftc.twentyfour.autotask;

public enum ObjectiveType {
    MOVEMENT_TIME, // run drive for an amount of time
    MOVEMENT_ENCODER, // run drive for a specific amount of ticks
    MOVEMENT_POSITION, // run drive until robot reaches a specific position (Maytag)
    DEVICE_SLIDE, // extend or retract the slide
    DEVICE_ARM, // lower or raise the claw arm
    DEVICE_CLAW // open or close the claw
}
