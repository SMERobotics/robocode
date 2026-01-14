package org.technodot.ftc.twentyfive.common;

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

    public Quaternion conjugate() {
        return new Quaternion(w, -x, -y, -z);
    }

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

    /**
     * Constructs a quaternion that rotates vectors expressed in the sensor frame into the robot frame.
     * Inputs are the robot frame unit axes expressed in the sensor frame: forward (+X_r), right (+Y_r), up (+Z_r).
     * Columns of matrix M = [forward right up] map robot-frame coords to sensor frame.
     * We need rotation sensor->robot, so we take R = M^T and convert R to quaternion.
     */
    public static Quaternion fromBasis(double[] forward, double[] right, double[] up) {
        // Orthonormalization safety (assumes roughly orthogonal already)
        forward = normalizeVec(forward);
        // Make right orthogonal to forward
        right = subtract(right, scale(forward, dot(forward, right)));
        right = normalizeVec(right);
        // Recompute up as forward x right (right-handed robot frame: X=forward, Y=right, Z=up)
        double[] newUp = cross(forward, right);
        double nUp = normVec(newUp);
        if (nUp < 1e-6) newUp = up; else newUp = normalizeVec(newUp);
        up = newUp;

        // Matrix M (sensor columns are robot axes in sensor frame)
        double m00 = forward[0]; double m01 = right[0]; double m02 = up[0];
        double m10 = forward[1]; double m11 = right[1]; double m12 = up[1];
        double m20 = forward[2]; double m21 = right[2]; double m22 = up[2];

        // R = M^T (sensor->robot)
        double r00 = m00, r01 = m10, r02 = m20;
        double r10 = m01, r11 = m11, r12 = m21;
        double r20 = m02, r21 = m12, r22 = m22;

        double trace = r00 + r11 + r22;
        double qw, qx, qy, qz;
        if (trace > 0) {
            double s = Math.sqrt(trace + 1.0) * 2; // s=4*qw
            qw = 0.25 * s;
            qx = (r21 - r12) / s;
            qy = (r02 - r20) / s;
            qz = (r10 - r01) / s;
        } else if (r00 > r11 && r00 > r22) {
            double s = Math.sqrt(1.0 + r00 - r11 - r22) * 2; // s=4*qx
            qw = (r21 - r12) / s;
            qx = 0.25 * s;
            qy = (r01 + r10) / s;
            qz = (r02 + r20) / s;
        } else if (r11 > r22) {
            double s = Math.sqrt(1.0 + r11 - r00 - r22) * 2; // s=4*qy
            qw = (r02 - r20) / s;
            qx = (r01 + r10) / s;
            qy = 0.25 * s;
            qz = (r12 + r21) / s;
        } else {
            double s = Math.sqrt(1.0 + r22 - r00 - r11) * 2; // s=4*qz
            qw = (r10 - r01) / s;
            qx = (r02 + r20) / s;
            qy = (r12 + r21) / s;
            qz = 0.25 * s;
        }
        return new Quaternion(qw, qx, qy, qz).normalize();
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]
        };
    }

    private static double dot(double[] a, double[] b) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }

    private static double normVec(double[] v) {
        return Math.sqrt(dot(v,v));
    }

    private static double[] normalizeVec(double[] v) {
        double n = normVec(v);
        if (n < 1e-12) return new double[]{0,0,0};
        return new double[]{
                v[0]/n,
                v[1]/n,
                v[2]/n
        };
    }

    private static double[] scale(double[] v, double s) {
        return new double[]{
                v[0]*s,
                v[1]*s,
                v[2]*s
        };
    }

    private static double[] subtract(double[] a, double[] b) {
        return new double[]{
                a[0]-b[0],
                a[1]-b[1],
                a[2]-b[2]
        };
    }

    @Override
    public String toString() {
        return String.format("q[%.5f, %.5f, %.5f, %.5f]", w,x,y,z);
    }
}
