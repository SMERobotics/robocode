package com.mech.ftc.twentyfive.defaults;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor frontRight;

    Camera camera = new Camera();

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        camera.init(hardwareMap);
    }

    public void drive(Gamepad gamepad) {
        float vertical = gamepad.left_stick_x;
        float horizontal = gamepad.left_stick_y;
        float pivot = gamepad.right_stick_x;

        float frontRightPower = vertical + horizontal - pivot;
        float backRightPower = vertical - horizontal + pivot;
        float frontLeftPower = vertical - horizontal - pivot;
        float backLeftPower = vertical + horizontal + pivot;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }

    public void driveToTag() {
        if(camera.idDetection() == 21) {
            
        } else if (camera.idDetection() == 22) {
            
        } else if (camera.idDetection() == 23) {
            
        }
    }
}
