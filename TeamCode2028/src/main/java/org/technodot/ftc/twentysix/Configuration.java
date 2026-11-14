package org.technodot.ftc.twentysix;

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

public class Configuration {
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
