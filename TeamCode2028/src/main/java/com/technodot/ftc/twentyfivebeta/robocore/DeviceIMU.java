package com.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.technodot.ftc.twentyfivebeta.Configuration;
import com.technodot.ftc.twentyfivebeta.common.Alliance;
import com.technodot.ftc.twentyfivebeta.common.Vector2D;
import com.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DeviceIMU extends Device<SilentRunner101> {

    public IMU imu;

    private final RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

    public static double headingOffset;
    public static double yaw;

    public DeviceIMU(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, SilentRunner101 inputController) {
        this.inputController = inputController;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        if (inputController.resetYaw()) zeroYaw();
        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        updateHeadingOffset();
    }

    @Override
    public void stop() {

    }

    public void updateHeadingOffset() {
        if (DeviceCamera.fieldOffset != null) DeviceCamera.fieldOffset.ifPresent(offset -> headingOffset = offset - yaw);
    }

    public double getHeadingOffset() {
        return headingOffset;
    }

    public void setHeadingOffset(double offset) {
        headingOffset = offset;
    }

    public static Vector2D rotateVector(Vector2D movement) {
        return movement.rotate(Math.toRadians(yaw - headingOffset));
//        return movement.rotate(Math.toRadians(yaw));
    }

    public void zeroYaw() {
        imu.resetYaw();
    }
}
