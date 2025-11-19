
package com.mech.ftc.twentyfive.defaults;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain {
    public DcMotorEx frontLeft;
    public DcMotorEx backRight;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx intake;
    public DcMotorEx launchMotor;
    public DcMotorEx indexMotor;
    public Servo kicker;
    public Servo wall;

    BNO055IMU imu;
    Orientation angles = new Orientation();
    double originalYaw;

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        indexMotor = hardwareMap.get(DcMotorEx.class, "indexMotor");
        kicker = hardwareMap.get(Servo.class, "servo");
        wall = hardwareMap.get(Servo.class, "servoTwo");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        indexMotor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        fieldOrientedVar(hardwareMap);
    }


    public void drive(Gamepad gamepad) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double headingRad = Math.toRadians(angles.firstAngle);

        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rx = gamepad.right_stick_x;

        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);
        double rotX = x * cos - y * sin;
        double rotY = x * sin + y * cos;

        double frontLeftPower = -rotY - rotX + rx;
        double frontRightPower = rotY - rotX + rx;
        double backLeftPower = rotY - rotX - rx;
        double backRightPower = -rotY - rotX - rx;


        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
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

    public void fieldOrientedVar(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize((parameters));

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        originalYaw = angles.firstAngle;
    }
}