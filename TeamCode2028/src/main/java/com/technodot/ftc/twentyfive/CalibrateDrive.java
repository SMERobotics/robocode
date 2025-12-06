package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="CalibrateDrive", group="TechnoCode")
public class CalibrateDrive extends OpMode {

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

    public float forward;
    public float strafe;
    public float rotate;

    public float speedMultiplier = 1.0F;

    private static final float DEADZONE = 0.02f;
    private static final float ACTIVATION = 0.2f;

    @Override
    public void init() {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        // toggle all of them to change robot drive direction
        motorFrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorEx.Direction.FORWARD); // ts should be reverse. CHAT MOUNTED IT BACKWARDS WHAT THE FUU
//        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void update(Gamepad gamepad) {
        update(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
    }

    public void update(float forward, float strafe, float rotate) {
        if (Math.abs(forward) < DEADZONE) forward = 0f;
        if (Math.abs(strafe) < DEADZONE) strafe = 0f;
        if (Math.abs(rotate) < DEADZONE) rotate = 0f;

        // robot-centric kinematics
        // TODO: field-centric kinematics
        // if it ain't broke, don't fix it
        // if it ain't broke, don't even flipping TOUCH it
        float fl = forward + strafe + rotate;
        float fr = forward - strafe - rotate;
        float bl = forward - strafe + rotate;
        float br = forward + strafe - rotate;

        // normalize
        float max = Math.max(1.0f, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        fl *= speedMultiplier;
        fr *= speedMultiplier;
        bl *= speedMultiplier;
        br *= speedMultiplier;

        update(fl, fr, bl, br);
    }

    public void update(float fl, float fr, float bl, float br) {
//        if (motorFrontLeft != null) motorFrontLeft.setPower(scaleInput(fl));
//        if (motorFrontRight != null) motorFrontRight.setPower(scaleInput(fr));
//        if (motorBackLeft != null) motorBackLeft.setPower(scaleInput(bl));
//        if (motorBackRight != null) motorBackRight.setPower(scaleInput(br));

        if (motorFrontLeft != null) motorFrontLeft.setPower(fl);
        if (motorFrontRight != null) motorFrontRight.setPower(fr);
        if (motorBackLeft != null) motorBackLeft.setPower(bl);
        if (motorBackRight != null) motorBackRight.setPower(br);
    }

    public void zero() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void getMultiplier(float multiplier) {
        speedMultiplier = multiplier;
    }

    public void setMultiplier(float multiplier) {
        speedMultiplier = multiplier;
    }

    public void resetMultiplier() {
        speedMultiplier = 1.0F;
    }

    private float scaleInput(float value) {
        if (value > 0) {
            return -value * ACTIVATION + value + ACTIVATION;
        } else if (value < 0) {
            return -value * ACTIVATION + value - ACTIVATION;
        } else {
            return 0;
        }
    }

    public void resetEncoders() {
        motorFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        resetEncoders();

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.8f) {
            resetEncoders();
        }

        if (gamepad1.right_trigger > 0.2f) {
            speedMultiplier = 0.5f;
        } else {
            speedMultiplier = 1.0f;
        }

        if (gamepad1.dpad_up) {
            forward += 0.01f;
        } else if (gamepad1.dpad_down) {
            forward -= 0.01f;
        }

        if (gamepad1.dpad_right) {
            strafe += 0.01f;
        } else if (gamepad1.dpad_left) {
            strafe -= 0.01f;
        }

        if (gamepad1.right_bumper) {
            rotate += 0.01f;
        } else if (gamepad1.left_bumper) {
            rotate -= 0.01f;
        }

        forward = Range.clip(forward, -1f, 1f);
        strafe = Range.clip(strafe, -1f, 1f);
        rotate = Range.clip(rotate, -1f, 1f);

        update(
            Range.clip(forward - gamepad1.left_stick_y, -1f, 1f),
            Range.clip(strafe + gamepad1.left_stick_x, -1f, 1f),
            Range.clip(rotate + gamepad1.right_stick_x, -1f, 1f)
        );

        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);

        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        telemetry.addData("fl", motorFrontLeft.getCurrentPosition());
        telemetry.addData("fr", motorFrontRight.getCurrentPosition());
        telemetry.addData("bl", motorBackLeft.getCurrentPosition());
        telemetry.addData("br", motorBackRight.getCurrentPosition());

        telemetry.addData("flv", motorFrontLeft.getVelocity());
        telemetry.addData("frv", motorFrontRight.getVelocity());
        telemetry.addData("blv", motorBackLeft.getVelocity());
        telemetry.addData("brv", motorBackRight.getVelocity());

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
