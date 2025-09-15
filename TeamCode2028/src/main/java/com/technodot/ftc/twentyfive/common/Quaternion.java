package com.technodot.ftc.twentyfive.common;

/**
 * Minimal immutable quaternion utility for attitude tracking.
 * Convention: quaternion represents rotation from the reference (world/initial) frame to the body/current frame.
 * Vector rotation (world -> body): v_body = q * v_world * q_conj
 * To express a body-frame vector in world frame: v_world = q_conj * v_body * q
 */
public class Quaternion {
    public final double w;
    public final double x;
    public final double y;
    public final double z;

    public static final Quaternion IDENTITY = new Quaternion(1,0,0,0);

    public Quaternion(double w, double x, double y, double z) {
        this.w = w; this.x = x; this.y = y; this.z = z;
    }

    public double norm() {
        return Math.sqrt(w*w + x*x + y*y + z*z);
    }

    public Quaternion normalize() {
        double n = norm();
        if (n == 0) return IDENTITY;
        return new Quaternion(w/n, x/n, y/n, z/n);
    }

    public Quaternion conjugate() { return new Quaternion(w, -x, -y, -z); }

    public Quaternion multiply(Quaternion o) {
        // (this * o)
        return new Quaternion(
                w*o.w - x*o.x - y*o.y - z*o.z,
                w*o.x + x*o.w + y*o.z - z*o.y,
                w*o.y - x*o.z + y*o.w + z*o.x,
                w*o.z + x*o.y - y*o.x + z*o.w
        );
    }

    /** Creates a quaternion from an angular velocity vector (rad/s) integrated over dt seconds. */
    public static Quaternion fromAngularVelocity(double wx, double wy, double wz, double dt) {
        double angle = Math.sqrt(wx*wx + wy*wy + wz*wz) * dt;
        if (angle < 1e-9) {
            // small-angle approximation
            double halfDt = 0.5 * dt;
            return new Quaternion(1.0, wx*halfDt, wy*halfDt, wz*halfDt).normalize();
        }
        double ax = wx; double ay = wy; double az = wz;
        double invMag = 1.0 / Math.sqrt(ax*ax + ay*ay + az*az);
        ax *= invMag; ay *= invMag; az *= invMag;
        double half = 0.5 * angle;
        double sinHalf = Math.sin(half);
        double cosHalf = Math.cos(half);
        return new Quaternion(cosHalf, ax*sinHalf, ay*sinHalf, az*sinHalf).normalize();
    }

    /** Rotate a body-frame vector into world frame (inverse rotation). */
    public double[] rotateBodyToWorld(double vx, double vy, double vz) {
        Quaternion qc = conjugate();
        // q_world = qc * v_body_quat * q
        Quaternion vq = new Quaternion(0, vx, vy, vz);
        Quaternion r = qc.multiply(vq).multiply(this);
        return new double[]{r.x, r.y, r.z};
    }

    /** Rotate a world-frame vector into body frame. */
    public double[] rotateWorldToBody(double vx, double vy, double vz) {
        Quaternion vq = new Quaternion(0, vx, vy, vz);
        Quaternion r = this.multiply(vq).multiply(this.conjugate());
        return new double[]{r.x, r.y, r.z};
    }

    public double[] toEulerRPY() {
        // Roll (x), Pitch (y), Yaw (z) following aerospace sequence.
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        double roll = Math.atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (w * y - z * x);
        double pitch = Math.abs(sinp) >= 1 ? Math.copySign(Math.PI / 2, sinp) : Math.asin(sinp);

        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = Math.atan2(siny_cosp, cosy_cosp);
        return new double[]{roll, pitch, yaw};
    }

    @Override
    public String toString() {
        return String.format("q[%.5f, %.5f, %.5f, %.5f]", w,x,y,z);
    }
}

