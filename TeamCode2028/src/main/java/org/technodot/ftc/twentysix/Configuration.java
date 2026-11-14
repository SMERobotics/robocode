package org.technodot.ftc.twentysix;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Configuration {
    public static MecanumConfig driveConfig = new MecanumConfig(
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
        c.xPodOffset.set(-1.0207723632572205);
        c.yPodOffset.set(-6.347154632328064);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static Follower create(HardwareMap h) {
        return null;
    }
}
