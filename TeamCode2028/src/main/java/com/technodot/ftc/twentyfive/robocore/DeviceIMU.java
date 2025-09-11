package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class DeviceIMU extends Device {
    public IMU imu;

    private final double[] offset = new double[]{0.0, 0.0, 0.0}; // where the IMU is relative to the robot, in meters
    // ex. IMU sits 0.05 m forward on robot, so offset is {0.05, 0.0, 0.0}


    private double[] prevOmega = new double[]{0.0, 0.0, 0.0};
    private long prevTime = -1L;

    @Override
    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));
        prevTime = System.nanoTime();
    }

    @Override
    public void update(Gamepad gamepad) {

    }

    // great. this is only relative psuedoacceleration. fuck this shit. fuck physics.
    public double[] update() {
        // read IMU data
        AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        Quaternion q = imu.getRobotOrientationAsQuaternion();
        double omegaX = av.xRotationRate;
        double omegaY = av.yRotationRate;
        double omegaZ = av.zRotationRate;
        double[] omega = new double[]{omegaX, omegaY, omegaZ};

        // compute dt
        long nowNs = System.nanoTime();
        double dt = (prevTime <= 0) ? 0.0 : (nowNs - prevTime) * 1e-9;
        double[] alpha = new double[]{0.0, 0.0, 0.0};
        if (dt > 0.0) {
            alpha[0] = (omega[0] - prevOmega[0]) / dt;
            alpha[1] = (omega[1] - prevOmega[1]) / dt;
            alpha[2] = (omega[2] - prevOmega[2]) / dt;
        }

        double[] alphaCrossR = cross(alpha, offset); // compute alpha x r (tangential)

        // compute omega x (omega x r) (centripetal)
        double[] omegaCrossR = cross(omega, offset);
        double[] omegaCrossOmegaCrossR = cross(omega, omegaCrossR);

        // rotational acceleration in BODY frame
        double[] aRotBody = new double[]{
                alphaCrossR[0] + omegaCrossOmegaCrossR[0],
                alphaCrossR[1] + omegaCrossOmegaCrossR[1],
                alphaCrossR[2] + omegaCrossOmegaCrossR[2]
        };

        double[] aInertial = rotate(aRotBody, q); // rotate aRotBody -> inertial

        prevOmega[0] = omega[0];
        prevOmega[1] = omega[1];
        prevOmega[2] = omega[2];
        prevTime = nowNs;

        return aInertial; // inertial frame in m/s^2
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        };
    }

    private static double[] rotate(double[] v, Quaternion q) {
        double w = q.w;
        double ux = q.x;
        double uy = q.y;
        double uz = q.z;

        // t = 2 * u x v
        double tx = 2.0 * (uy * v[2] - uz * v[1]);
        double ty = 2.0 * (uz * v[0] - ux * v[2]);
        double tz = 2.0 * (ux * v[1] - uy * v[0]);

        // v' = v + w * t + u x t
        double[] uCrossT = cross(new double[]{ux, uy, uz}, new double[]{tx, ty, tz});

        return new double[]{
                v[0] + w * tx + uCrossT[0],
                v[1] + w * ty + uCrossT[1],
                v[2] + w * tz + uCrossT[2]
        };
    }
}
