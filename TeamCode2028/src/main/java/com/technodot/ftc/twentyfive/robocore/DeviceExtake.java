package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeviceExtake extends Device {
    public DcMotor motorExtake;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorExtake = hardwareMap.get(DcMotor.class, "motorExtake");
        motorExtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {

    }

    @Override
    public void update(Gamepad gamepad) {
//        motorExtake.setPower(gamepad.right_trigger - gamepad.left_trigger);

        // TODO: allow for dynamic acceleration!!!

//        TechnoDot
//        — 12:18 PM
//        @Silent Cyber
//        about the robotics controls
//        im doing that right now
//        so motor extake speed of 0.67, it takes too long to accelerate
//        it needs to start off at 1 to accelerate faster, and then lower down to 0.67 when it has reached the target speed
//        how would you like me do to that
//        would you like me to 1. designate the range between 0.2 and 0.8 or smth on the triggers as 1.0, and then above 0.8, lower the speed to 0.67
//        OR just have it handled automatically by the robot, but you lose that freedom; when you activate the extake too fast in between, it may accelerate to above 0.67
//        i MIGHT be able to have it handle the speed automatically through encoders, but i'm not sure on that yet
//        if i can, then i will do that

        if (gamepad.right_trigger > 0.2) {
            motorExtake.setPower(0.67);
        } else {
            motorExtake.setPower(0);
        }
    }

    public void update(double power) {
        motorExtake.setPower(power);
    }

    @Override
    public void stop() {

    }
}
