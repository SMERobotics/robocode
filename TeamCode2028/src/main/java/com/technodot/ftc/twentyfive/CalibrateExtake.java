package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.technodot.ftc.twentyfive.robocore.DeviceIntake;

@TeleOp(name="CalibrateExtake", group="TechnoCode")
public class CalibrateExtake extends OpMode {

    public DcMotorEx motorExtake;
    public DeviceIntake deviceIntake = new DeviceIntake();
    public float power = 0.0f;

    @Override
    public void init() {
        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        deviceIntake.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        power = 0.0f;

        motorExtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        motorExtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        deviceIntake.update(gamepad1);
        deviceIntake.report(telemetry);

        if (gamepad1.dpad_up) {
            power += 0.01f;
        } else if (gamepad1.dpad_down) {
            power -= 0.01f;
        }
        power = Range.clip(power, -1.0f, 1.0f);

        motorExtake.setPower(power);
        telemetry.addData("power", power);
        telemetry.addData("position", motorExtake.getCurrentPosition());
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
