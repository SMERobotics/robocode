package org.technodot.ftc.twentysix.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.technodot.ftc.twentysix.robocore.DeviceExtake;

@TeleOp(name="TestExtake", group="TechnoCode")
public class TestExtake extends OpMode {

    public DcMotorEx motorExtake;
    public Servo servoExtake;

    @Override
    public void init() {
        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setDirection(DcMotorEx.Direction.REVERSE);
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        servoExtake = hardwareMap.get(Servo.class, "servoExtake");
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.addData("servo_pos", servoExtake.getPosition());
        telemetry.update();
    }

    @Override
    public void start() {
        motorExtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorExtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        servoExtake.setPosition(DeviceExtake.SERVO_NECTAR_POS);

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            motorExtake.setPower(1.0);
        } else if (gamepad1.right_trigger > 0.0) {
            motorExtake.setPower(gamepad1.right_trigger);
        } else {
            double motorAdjustment = (gamepad1.dpad_up ? 0.001 : 0.0)
                    - (gamepad1.dpad_down ? 0.001 : 0.0);
            motorExtake.setPower(Math.max(-1.0, Math.min(1.0,
                    motorExtake.getPower() + motorAdjustment)));
        }

        if (gamepad1.a) {
            servoExtake.setPosition(DeviceExtake.SERVO_POLLEN_POS);
        } else if (gamepad1.b) {
            servoExtake.setPosition(DeviceExtake.SERVO_NECTAR_POS);
        }

        telemetry.addData("servo_pos", servoExtake.getPosition());
        telemetry.addData("pwr", motorExtake.getPower());
//        telemetry.addData("pos", motorExtake.getCurrentPosition());
//        telemetry.addData("vel", motorExtake.getVelocity());

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
