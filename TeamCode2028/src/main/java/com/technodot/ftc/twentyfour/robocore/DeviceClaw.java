package com.technodot.ftc.twentyfour.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.technodot.ftc.twentyfour.Configuration;

public class DeviceClaw extends Device {
    public Servo clawMain;
    public DcMotor clawArm;

    public int armStatus = 0;
    public float speedMultiplier = 1.0F;

    public DeviceClaw(HardwareMap hardwareMap) {
        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        clawMain = hardwareMap.get(Servo.class, "clawMain");
        // clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void updatePreciseMode(boolean preciseMode) {
        if (preciseMode) {
            speedMultiplier = Configuration.armPreciseModeMultiplier;
        } else {
            speedMultiplier = 1.0F;
        }
    }

    public void resetArmEncoder() {
        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void updateClaw(boolean clawOpen, boolean clawClose) {
        if (clawOpen) {
            clawMain.setPosition(Configuration.clawOpenPosition);
        } else if (clawClose) {
            clawMain.setPosition(Configuration.clawClosePosition);
        }
    }

    public void updateArm(boolean armOpen, boolean armClose) {
        int targetPosition;

        if (armOpen) {
            targetPosition = Configuration.armOpenPosition;
            armStatus = 1;
            clawArm.setTargetPosition(targetPosition);
            clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (armClose) {
            targetPosition = Configuration.armClosePosition;
            armStatus = -1;
            clawArm.setTargetPosition(targetPosition);
            clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        clawArm.setPower(armStatus);
    }

    public void updateArmForce(boolean armCloseForce) {
        if (armCloseForce) {
            clawArm.setPower(-1.0F);
        }
    }
}
