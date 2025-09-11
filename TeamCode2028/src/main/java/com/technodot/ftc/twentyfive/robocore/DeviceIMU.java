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

    // Set this to the vector from the rotation origin to the point of interest, expressed in body (IMU) frame (meters).
    // Example: IMU sits 0.05 m forward on robot, so sensorOffset = {0.05, 0.0, 0.0}
    private double[] sensorOffset = new double[]{0.0, 0.0, 0.0};

    // store previous angular velocity and time for finite-difference alpha
    private double[] prevOmega = new double[]{0.0, 0.0, 0.0};
    private long prevTimeNs = -1L;

    @Override
    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // initialize prevTimeNs so first delta will be skipped/ignored
        prevTimeNs = System.nanoTime();
    }

    /** Optional setter for the sensor offset (in meters, body frame) */
    public void setSensorOffsetMeters(double x, double y, double z) {
        sensorOffset[0] = x; sensorOffset[1] = y; sensorOffset[2] = z;
    }

    @Override
    public void update(Gamepad gamepad) {

    }

    /**
     * Returns acceleration of the point (sensorOffset) in the INERTIAL frame (m/s^2),
     * computed from angular velocity/acceleration and the offset.
     */
    public double[] update() {
        // 1) read angular velocity (body frame) from IMU in radians/sec
        AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        double omegaX = av.xRotationRate;
        double omegaY = av.yRotationRate;
        double omegaZ = av.zRotationRate;
        double[] omega = new double[]{omegaX, omegaY, omegaZ};

        // 2) compute dt (seconds)
        long nowNs = System.nanoTime();
        double dt = (prevTimeNs <= 0) ? 0.0 : (nowNs - prevTimeNs) * 1e-9;
        // if dt is zero (first sample), set alpha = 0 to avoid divide-by-zero
        double[] alpha = new double[]{0.0, 0.0, 0.0};
        if (dt > 0.0) {
            alpha[0] = (omega[0] - prevOmega[0]) / dt;
            alpha[1] = (omega[1] - prevOmega[1]) / dt;
            alpha[2] = (omega[2] - prevOmega[2]) / dt;
        }

        // 3) compute alpha x r  (tangential)
        double[] alphaCrossR = cross(alpha, sensorOffset);

        // 4) compute omega x (omega x r)  (centripetal)
        double[] omegaCrossR = cross(omega, sensorOffset);
        double[] omegaCrossOmegaCrossR = cross(omega, omegaCrossR);

        // rotational acceleration in BODY frame
        double[] aRotBody = new double[]{
                alphaCrossR[0] + omegaCrossOmegaCrossR[0],
                alphaCrossR[1] + omegaCrossOmegaCrossR[1],
                alphaCrossR[2] + omegaCrossOmegaCrossR[2]
        };

        // 5) rotate aRotBody -> inertial using the IMU quaternion
        Quaternion q = imu.getRobotOrientationAsQuaternion(); // quaternion from body -> inertial
        double[] aInertial = rotateVectorByQuaternion(aRotBody, q);

        // store for next iteration
        prevOmega[0] = omega[0]; prevOmega[1] = omega[1]; prevOmega[2] = omega[2];
        prevTimeNs = nowNs;

        // return acceleration in inertial frame (m/s^2)
        return aInertial;
    }

    // ---- helper math functions ----

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]
        };
    }

    /**
     * Rotate vector v_body by quaternion q to get vector in inertial frame.
     * Uses quaternion formula where q = [w, (ux,uy,uz)].
     *
     * Note: field names on your Quaternion class may differ (some versions use w,x,y,z).
     * If your Quaternion uses a different order, adapt accordingly.
     */
    private static double[] rotateVectorByQuaternion(double[] v, Quaternion q) {
        // assume q has fields w, x, y, z (typical in FTC libraries)
        double w = q.w;
        double ux = q.x;
        double uy = q.y;
        double uz = q.z;

        // t = 2 * u x v
        double tx = 2.0 * (uy * v[2] - uz * v[1]);
        double ty = 2.0 * (uz * v[0] - ux * v[2]);
        double tz = 2.0 * (ux * v[1] - uy * v[0]);

        // v' = v + w * t + u x t
        double[] u = new double[]{ux, uy, uz};
        double[] uCrossT = cross(u, new double[]{tx, ty, tz});

        return new double[]{
                v[0] + w * tx + uCrossT[0],
                v[1] + w * ty + uCrossT[1],
                v[2] + w * tz + uCrossT[2]
        };
    }
}
