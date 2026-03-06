package org.technodot.ftc.twentyfiveoff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="TestXDrive", group="TechnoCode")
public class TestXDrive extends OpMode {

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

    @Override
    public void init() {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorFrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorEx.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double fl = rotate + forward + strafe;
        double fr = rotate - forward + strafe;
        double bl = rotate + forward - strafe;
        double br = rotate - forward - strafe;

        double max = Math.max(1.0f, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
        motorBackLeft.setPower(bl);
        motorBackRight.setPower(br);


        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
