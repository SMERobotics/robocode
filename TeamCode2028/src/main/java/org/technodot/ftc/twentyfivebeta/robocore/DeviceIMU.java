package org.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Vector2D;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DeviceIMU extends Device {

//    public IMU imu;
    public Rev9AxisImu rev;

//    private final RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    private final Rev9AxisImuOrientationOnRobot revOrientationOnRobot = new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.DOWN, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.RIGHT);

    public static double headingOffset;
    public static double yaw;
    public static double targetYaw;
    public static double timeNs;

    public static final double GOAL_DEG = Math.toDegrees(Math.atan((double) 4 / 3));

    public DeviceIMU(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        this.inputController = inputController;

//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(orientationOnRobot));

        rev = hardwareMap.get(Rev9AxisImu.class, "rev");
        rev.initialize(new Rev9AxisImu.Parameters(revOrientationOnRobot));
    }

    @Override
    public void start() {
        headingOffset = 0;
        DeviceIMU.setSnapshotYaw();
    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;
        if (ctrl.resetYaw()) zeroYaw();
//        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        yaw = rev.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + headingOffset; // previously was -, does ts cause problems?
        timeNs = System.nanoTime();
//        updateHeadingOffset();
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
//        return movement.rotate(Math.toRadians(yaw - headingOffset));
        return movement.rotate(Math.toRadians(yaw));
    }

    public void zeroYaw() {
//        imu.resetYaw();
        targetYaw -= rev.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        headingOffset = 0;
        rev.resetYaw();
    }

    public static void setSnapshotYaw() {
        targetYaw = yaw;
    }

    public static double getSnapshotYawError() {
        double error = targetYaw - yaw;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}
