package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="CalibrateExtake", group="TechnoCode")
public class CalibrateExtake extends OpMode {

    public DcMotorEx motorExtake;
    public float power = 0.0f;

    @Override
    public void init() {
        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        power = 0.0f;

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            power += 0.01f;
        } else if (gamepad1.dpad_down) {
            power -= 0.01f;
        }
        power = Math.max(-1.0f, Math.min(1.0f, power));

        motorExtake.setPower(power);
        telemetry.addData("power", power);
        telemetry.addData("velocity", motorExtake.getVelocity());

        // !!! pretend power means velocity here
//        motorExtake.setVelocity(power);
//        telemetry.addData("power", motorExtake.getPower());
//        telemetry.addData("velocity", power);

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
