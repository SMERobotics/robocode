package org.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Vector2D;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

public class DevicePinpoint extends Device {

    public static GoBildaPinpointDriver pinpoint;

    public static double snapshotYaw;

    public DevicePinpoint(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        this.inputController = inputController;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(Configuration.PINPOINT_DIRECTION_FORWARD, Configuration.PINPOINT_DIRECTION_STRAFE);
        pinpoint.setOffsets(Configuration.PINPOINT_OFFSET_FORWARD_Y, Configuration.PINPOINT_OFFSET_STRAFE_X, DistanceUnit.INCH);
    }

    @Override
    public void start() {
        reset();
        // setPose() SHOULD BE CALLED AFTER THIS IN AUTO INIT
    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;

        if (ctrl.resetYaw()) resetIMU();
        if (ctrl.resetPos()) reset();

        this.updatePinpoint(); // TODO: call in thread
    }

    @Override
    public void stop() {

    }

    public static Vector2D rotateVector(Vector2D movement) {
        return movement.rotate(Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES) + (alliance == Alliance.BLUE ? -90 : 90)));
    }

    public void setPose(double x, double y, double h) {
        pinpoint.setPosX(x, DistanceUnit.INCH);
        pinpoint.setPosY(y, DistanceUnit.INCH);
        pinpoint.setHeading(h, AngleUnit.DEGREES);
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
        // setPose() SHOULD BE CALLED AFTER THIS IN AUTO INIT
    }

    public void resetIMU() {
//        pinpoint.recalibrateIMU();
        pinpoint.setHeading(0, AngleUnit.DEGREES);
    }

    public static void setSnapshotYaw() {
        snapshotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public static double getSnapshotYawError() {
        double error = snapshotYaw - pinpoint.getHeading(AngleUnit.DEGREES);
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    private void updatePinpoint() {
        pinpoint.update();
    }
}
