package com.mech.ftc.twentyfive.defaults;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveTrain {
    public DcMotorEx frontLeft;
    public DcMotorEx backRight;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx intake;
    public DcMotorEx launchMotor;
    public DcMotorEx indexMotor;
    public Servo kicker;

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        indexMotor = hardwareMap.get(DcMotorEx.class, "indexMotor");
        kicker = hardwareMap.get(Servo.class, "servo");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(Gamepad gamepad) {
        float vertical = gamepad.left_stick_y;
        float horizontal = gamepad.left_stick_x;
        float pivot = gamepad.right_stick_x;

        float frontRightPower = vertical + horizontal + pivot;
        float backRightPower = -vertical + horizontal - pivot;
        float frontLeftPower = -vertical + horizontal + pivot;
        float backLeftPower = vertical + horizontal - pivot;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }
    public void drive(float vertical, float horizontal, float pivot) {
        float frontRightPower = vertical + horizontal + pivot;
        float backRightPower = -vertical + horizontal - pivot;
        float frontLeftPower = -vertical + horizontal + pivot;
        float backLeftPower = vertical + horizontal - pivot;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }
}
