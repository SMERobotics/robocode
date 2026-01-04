package org.technodot.ftc.twentyfivebeta.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

@TeleOp(name="CalibrateExtakeMotor", group="TechnoCode")
public class CalibrateExtakeMotor extends OpMode {

    public DeviceExtake deviceExtake;

    private double velocity = 0;

    @Override
    public void init() {
        deviceExtake = new DeviceExtake(Alliance.BLUE);
        deviceExtake.init(hardwareMap, new SilentRunner101(null, null));
        deviceExtake.setExtakeState(DeviceExtake.ExtakeState.OVERRIDE);


    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        deviceExtake.start();

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            velocity += 1;
        } else if (gamepad1.dpad_down) {
            velocity -= 1;
        }

        deviceExtake.setExtakeOverride(velocity * 20);

        deviceExtake.update();

        telemetry.addData("ext", velocity * 20);
        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        deviceExtake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
