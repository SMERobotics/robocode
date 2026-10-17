package org.technodot.ftc.twentysixseven;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.technodot.ftc.twentyfivebeta.Configuration;

@Disabled
@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends OpMode {

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

    public DcMotorEx motorIntakeLeft;
    public DcMotorEx motorIntakeRight;
    public DcMotorEx motorExtake;

    public Servo servoTransfer;

    public double transferPos = 0.0;

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

        motorIntakeLeft = hardwareMap.get(DcMotorEx.class, "motorIntakeLeft");
        motorIntakeRight = hardwareMap.get(DcMotorEx.class, "motorIntakeRight");
        motorIntakeLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorIntakeRight.setDirection(DcMotorEx.Direction.FORWARD);
        motorIntakeLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorIntakeRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setDirection(DcMotorEx.Direction.FORWARD);
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        servoTransfer = hardwareMap.get(Servo.class, "servoTransfer");
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
        double strafe = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(1.0f, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        fl *= Configuration.DRIVE_SPEED_MULTIPLIER;
        fr *= Configuration.DRIVE_SPEED_MULTIPLIER;
        bl *= Configuration.DRIVE_SPEED_MULTIPLIER;
        br *= Configuration.DRIVE_SPEED_MULTIPLIER;

        if (motorFrontLeft != null) motorFrontLeft.setPower(fl);
        if (motorFrontRight != null) motorFrontRight.setPower(fr);
        if (motorBackLeft != null) motorBackLeft.setPower(bl);
        if (motorBackRight != null) motorBackRight.setPower(br);

        motorIntakeLeft.setPower(gamepad1.right_trigger);
        motorIntakeRight.setPower(gamepad1.right_trigger);

        motorExtake.setPower(gamepad1.left_trigger);

        if (gamepad1.a) {
            transferPos = 0.0;
        }
        if (gamepad1.b) {
            transferPos = 0.5;
        }
        servoTransfer.setPosition(transferPos);

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
