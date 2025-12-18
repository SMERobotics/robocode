package org.technodot.ftc.twentyfour;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.technodot.ftc.twentyfour.maytag.MayTag;
import org.technodot.ftc.twentyfour.robocore.DeviceClaw;
import org.technodot.ftc.twentyfour.robocore.DeviceDrive;
import org.technodot.ftc.twentyfour.robocore.DeviceSlide;

@Disabled
@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends LinearOpMode {

    // controllers
    DeviceDrive deviceDrive;
    DeviceClaw deviceClaw;
    DeviceSlide deviceSlide;
    MayTag mayTag;

    public void runAuto() {
        deviceDrive.runDrive(-1.0F, -1.0F, 1.0F);
    }

    @Override
    public void runOpMode() {
        // initialize devices
        deviceDrive = new DeviceDrive(hardwareMap, Configuration.driveTrainType, Configuration.driveControlType);
        deviceClaw = new DeviceClaw(hardwareMap);
        deviceSlide = new DeviceSlide(hardwareMap);

        // initialize MayTag library
        if (Configuration.ENABLE_AUTO_MAYTAG) {
            mayTag = new MayTag(hardwareMap, telemetry);
        }

        // finish initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runAuto();

        while (opModeIsActive()) {
            if (Configuration.ENABLE_AUTO_MAYTAG) {
                mayTag.update();
            }

            telemetry.update();
        }

        if (Configuration.ENABLE_AUTO_MAYTAG) {
            mayTag.close();
        }

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}