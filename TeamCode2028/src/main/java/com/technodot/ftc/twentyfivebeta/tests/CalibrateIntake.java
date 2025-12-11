package com.technodot.ftc.twentyfivebeta.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp(name="CalibrateIntake", group="TechnoCode")
public class CalibrateIntake extends OpMode {

    public RevColorSensorV3 colorLeft1;
    public RevColorSensorV3 colorLeft2;
    public RevColorSensorV3 colorRight1;
    public RevColorSensorV3 colorRight2;

    public HashMap<String, RevColorSensorV3> colorSensors;

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {
        colorLeft1 = hardwareMap.get(RevColorSensorV3.class, "colorLeft1");
        colorLeft2 = hardwareMap.get(RevColorSensorV3.class, "colorLeft2");
//        colorRight1 = hardwareMap.get(RevColorSensorV3.class, "colorRight1");
//        colorRight2 = hardwareMap.get(RevColorSensorV3.class, "colorRight2");

        if (colorLeft1 != null) colorSensors.put("cl1", colorLeft1);
        if (colorLeft2 != null) colorSensors.put("cl2", colorLeft2);
        if (colorRight1 != null) colorSensors.put("cr1", colorRight1);
        if (colorRight2 != null) colorSensors.put("cr2", colorRight2);

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
        colorSensors.forEach((k, v) -> {
            telemetry.addData(k + "_c", v.getNormalizedColors());
            telemetry.addData(k + "_d", v.getDistance(DistanceUnit.CM));
            telemetry.addData(k + "_l", v.getLightDetected());
        });

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
