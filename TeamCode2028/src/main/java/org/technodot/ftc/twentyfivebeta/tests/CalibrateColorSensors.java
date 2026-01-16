package org.technodot.ftc.twentyfivebeta.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.HashMap;

@Disabled
@TeleOp(name="CalibrateColorSensors", group="TechnoCode")
public class CalibrateColorSensors extends OpMode {

    public DeviceIntake deviceIntake;

    public RevColorSensorV3 colorLeft1;
    public RevColorSensorV3 colorLeft2;
    public RevColorSensorV3 colorRight1;
    public RevColorSensorV3 colorRight2;

    public HashMap<String, RevColorSensorV3> colorSensors = new HashMap<>();

    public Telemetry t;

    @Override
    public void init() {
        this.t = FtcDashboard.getInstance().getTelemetry();

        deviceIntake = new DeviceIntake(Alliance.BLUE);
        deviceIntake.init(hardwareMap, new SilentRunner101(gamepad1, gamepad2));

        colorLeft1 = hardwareMap.get(RevColorSensorV3.class, "colorLeft1");
        colorLeft2 = hardwareMap.get(RevColorSensorV3.class, "colorLeft2");
        colorRight1 = hardwareMap.get(RevColorSensorV3.class, "colorRight1");
        colorRight2 = hardwareMap.get(RevColorSensorV3.class, "colorRight2");
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        deviceIntake.start();

        if (colorLeft1 != null) colorSensors.put("cl1", colorLeft1);
        if (colorLeft2 != null) colorSensors.put("cl2", colorLeft2);
        if (colorRight1 != null) colorSensors.put("cr1", colorRight1);
        if (colorRight2 != null) colorSensors.put("cr2", colorRight2);

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        deviceIntake.update();

        colorSensors.forEach((k, v) -> {
            NormalizedRGBA color = v.getNormalizedColors();
            double distanceCM = v.getDistance(DistanceUnit.CM);
            telemetry.addData(k + "_c", String.format("r=%.2f g=%.2f b=%.2f a=%.2f", color.red, color.green, color.blue, color.alpha));
            t.addData(k + "_cr", color.red);
            t.addData(k + "_cg", color.green);
            t.addData(k + "_cb", color.blue);
            t.addData(k + "_ca", color.alpha);
            t.addData(k + "_d", distanceCM);
            t.addData(k + "_l", v.getLightDetected());
            t.addData(k + "_a", DeviceIntake.getArtifactColor(color));
            telemetry.addData(k + "_a", DeviceIntake.getArtifactColor(color));
        });

        t.update();
        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        deviceIntake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
