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

    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;

        if (ctrl.recalibratePinpoint()) pinpoint.resetPosAndIMU(); // see below crashout
        if (ctrl.resetYaw()) resetIMU();

        this.updatePinpoint(); // TODO: call in thread
    }

    @Override
    public void stop() {

    }

    public static Vector2D rotateVector(Vector2D movement) {
        return movement.rotate(Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES)));
    }

    public void setPose(double x, double y, double h) {
        pinpoint.setPosX(x, DistanceUnit.INCH);
        pinpoint.setPosY(y, DistanceUnit.INCH);
        pinpoint.setHeading(h, AngleUnit.DEGREES);
    }

    public static Vector2D getPos() {
        return new Vector2D(pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getPosX(DistanceUnit.INCH));
    }

    public static void setPos(Vector2D pos) {
        pinpoint.setPosX(pos.y, DistanceUnit.INCH);
        pinpoint.setPosY(pos.x, DistanceUnit.INCH);
    }

    /*
     * WHAT THE FUCK GOBILDA
     * WHAT THE ACTUAL FUCK
     *
     * RESET
     * and
     * RECALIBRATE
     * are
     * TWO COMPLETELY DIFFERENT FUCKING THINGS
     *
     * THEY ARE **NOT** INTERCHANGEABLE
     */

//    public void reset() {
//        // NAH WTF GOBILDA
//        // IM NOT LETTING TS SLIDE
//
////        pinpoint.resetPosAndIMU();
//
//        // setPose() SHOULD BE CALLED AFTER THIS IN AUTO INIT
//    }

    /**
     * THE ROBOT MUST BE COMPLETELY STILL ON AUTO INIT OR ELSE YOU'RE GETTING TOUCHED
     */
    public void recalibrate() {
        pinpoint.resetPosAndIMU();
    }

    public void resetIMU() {
//        pinpoint.recalibrateIMU();

        snapshotYaw -= pinpoint.getHeading(AngleUnit.DEGREES);
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

    public static double getGoalYawError() {
        Vector2D goal = getGoalPos();

        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.getPosY(DistanceUnit.INCH);

        double targetAngle = Math.toDegrees(Math.atan2(goal.y - currentY, goal.x - currentX)) + 180;
        double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        double error = targetAngle - currentHeading;

        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }

    public static double getGoalDistance() {
        Vector2D goal = getGoalPos();

        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.getPosY(DistanceUnit.INCH);

        return Math.hypot(goal.x - currentX, goal.y - currentY);
    }

    private static Vector2D getGoalPos() {
        // TODO: adjust goal pos based on velocity and DeviceIntake.intakeSide
        return new Vector2D(alliance == Alliance.BLUE ? 0 : 144, 144);
    }

    private void updatePinpoint() {
        // TODO: move into distinct thread
        pinpoint.update();
    }
}
