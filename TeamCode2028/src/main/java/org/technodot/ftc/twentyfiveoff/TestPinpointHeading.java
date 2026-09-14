package org.technodot.ftc.twentyfiveoff;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TestPinpointHeading", group = "TechnoCode")
public class TestPinpointHeading extends OpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            pinpoint.setHeading(0, AngleUnit.DEGREES);
        }

        if (gamepad1.bWasPressed()) {
            pinpoint.recalibrateIMU();
        }

        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        telemetry.addData("h", pinpoint.getHeading(AngleUnit.DEGREES));
    }
}
