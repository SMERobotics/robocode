package org.technodot.ftc.twentysix;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Configuration {
    public static volatile double DRIVE_SPEED_MULTIPLIER = 1.0;
    public static volatile double EXTAKE_READY_VELOCITY_TOLERANCE = 40;
    public static volatile long EXTAKE_READY_SETTLE_MS = 300;
    public static volatile int EXTAKE_READY_RUMBLE_MS = 200;
    // Robot-center offsets; the 8-inch rear half-length places the chassis against its starting wall.
    public static double AUTO_CENTER_FROM_ALLIANCE_WALL = 24 + 8;
    public static double AUTO_CENTER_FROM_STARTING_WALL = 8;
    public static double AUTO_MOVE_RIGHT_INCHES = 3;
    public static double AUTO_SHOOT_VELOCITY = 1650;
    // Vision steers the turret, but a missing frame must not prevent the four-shot sequence.
    public static volatile boolean AUTO_REQUIRE_HIVE_VISION = false;
    public static double AUTO_VELOCITY_TOLERANCE = 40;
    public static double AUTO_TURRET_TOLERANCE_DEGREES = 2;
    public static double AUTO_POSITION_TOLERANCE_INCHES = 0.5;
    public static double AUTO_HEADING_TOLERANCE_DEGREES = 2;
    public static long AUTO_MOVE_TIMEOUT_MS = 4000;
    public static long AUTO_READY_TIMEOUT_MS = 4000;
    public static long AUTO_READY_SETTLE_MS = 150;
    // Four timed feed pulses; tune the duration to pass exactly one pollen on this mechanism.
    public static long AUTO_FEED_MS = 150;
    public static long AUTO_BETWEEN_SHOTS_MS = 400;

    // Negative values load the hub's existing coefficients on the first init.
    public static volatile double EXTAKE_VELOCITY_KP = -1;
    public static volatile double EXTAKE_VELOCITY_KI = -1;
    public static volatile double EXTAKE_VELOCITY_KD = -1;
    public static volatile double EXTAKE_VELOCITY_KF = -1;
    public static volatile double TURRET_VELOCITY_KP = 11;
    public static volatile double TURRET_VELOCITY_KI = -1;
    public static volatile double TURRET_VELOCITY_KD = -1;
    public static volatile double TURRET_VELOCITY_KF = -1;
    // The SDK's RUN_TO_POSITION controller supports only P; it uses the velocity PIDF internally.
    public static volatile double TURRET_POSITION_KP = -1;

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
        c -> {
            c.frontLeftName.set("motorFrontLeft");
            c.backLeftName.set("motorBackLeft");
            c.frontRightName.set("motorFrontRight");
            c.backRightName.set("motorBackRight");

            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(-0.697003000000);
        c.yPodOffset.set(-5.654728600000);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.28899297285354747);
            Controller secondaryTranslationalForward = Controller.proportional(0.1067751814349324);
            Controller primaryTranslationalLateral = Controller.proportional(0.41053504429485493);
            Controller secondaryTranslationalLateral = Controller.proportional(0.1516817291685335);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.017976826421022336));
            c.brake.set(Controller.proportionalFeedforward(0.015280302457868985));

            c.headingFeedback.set(Controller.proportional(6.533932130997095));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07137998919785843, 0.0026750903975834963));

            c.linearBrakeCoefficients.set(Matrix.diag(0.07297977569148398, 0.0832463144942206));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0019721827952651114, 0.00155902048463954));

            c.maxAchievableForwardVelocity.set(62.83646086922646);
            c.maxAchievableStrafeVelocity.set(47.012510056457366);
            c.naturalForwardDeceleration.set(51.66013056309286);
            c.naturalStrafeDeceleration.set(89.4843810830349);
        }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
            new PinpointLocalizer(h, localizerConfig),
            new Mecanum(h, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }
}
