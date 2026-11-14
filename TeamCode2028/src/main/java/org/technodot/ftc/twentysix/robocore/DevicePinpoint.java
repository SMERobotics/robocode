package org.technodot.ftc.twentysix.robocore;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.technodot.ftc.twentysix.Configuration;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;

public class DevicePinpoint extends Device {
    public GoBildaPinpointDriver pinpoint;

    public DevicePinpoint(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, GamepadBindings gamepad, Telemetry telemetry) {
        super.init(hardwareMap, gamepad, telemetry);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Configuration.localizerConfig.name.get());
        pinpoint.setEncoderResolution(Configuration.localizerConfig.podType.get());
        pinpoint.setEncoderDirections(Configuration.localizerConfig.xPodDirection.get(), Configuration.localizerConfig.yPodDirection.get());
        pinpoint.setOffsets(Configuration.localizerConfig.xPodOffset.get(), Configuration.localizerConfig.yPodOffset.get(), Configuration.localizerConfig.offsetUnits.get());
//        pinpoint.resetPosAndIMU();
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        pinpoint.update();
        telemetry.addData("pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("pinpoint x (in)", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("pinpoint y (in)", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("pinpoint heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
    }

    @Override
    public void stop() {
    }
}
