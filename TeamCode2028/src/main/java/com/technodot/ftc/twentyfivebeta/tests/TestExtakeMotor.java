package com.technodot.ftc.twentyfivebeta.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="TestExtakeMotor", group="TechnoCode")
public class TestExtakeMotor extends OpMode {

    public DcMotorEx motorExtakeLeft;
    public DcMotorEx motorExtakeRight;

    @Override
    public void init() {
        motorExtakeLeft = hardwareMap.get(DcMotorEx.class, "motorExtakeLeft");
        motorExtakeRight = hardwareMap.get(DcMotorEx.class, "motorExtakeRight");
        motorExtakeLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorExtakeRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorExtakeLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorExtakeRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        motorExtakeLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorExtakeLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorExtakeRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorExtakeRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left) {
            motorExtakeLeft.setVelocity(670);
        } else {
            motorExtakeLeft.setVelocity(0);
        }

        if (gamepad1.dpad_right) {
            motorExtakeRight.setVelocity(670);
        } else {
            motorExtakeRight.setVelocity(0);
        }

        telemetry.addData("l_pwr", motorExtakeLeft.getPower());
        telemetry.addData("l_pos", motorExtakeLeft.getCurrentPosition());
        telemetry.addData("l_vel", motorExtakeLeft.getVelocity());

        telemetry.addData("r_pwr", motorExtakeRight.getPower());
        telemetry.addData("r_pos", motorExtakeRight.getCurrentPosition());
        telemetry.addData("r_vel", motorExtakeRight.getVelocity());

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
