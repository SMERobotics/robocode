package org.technodot.ftc.twentysix.roboctrl;

final class HiveMath {
    static double[][] rx(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new double[][] {{1, 0, 0}, {0, c, -s}, {0, s, c}};
    }

    static double[][] ry(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new double[][] {{c, 0, s}, {0, 1, 0}, {-s, 0, c}};
    }

    static double[][] rz(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new double[][] {{c, -s, 0}, {s, c, 0}, {0, 0, 1}};
    }

    static double[][] multiply(double[][] a, double[][] b) {
        double[][] c = new double[3][3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) c[i][j] += a[i][k] * b[k][j];
        }
        return c;
    }

    static double[] multiply(double[][] a, double[] b) {
        double[] c = new double[3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) c[i] += a[i][j] * b[j];
        return c;
    }

    static double[][] transpose(double[][] a) {
        double[][] b = new double[3][3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) b[i][j] = a[j][i];
        return b;
    }

    static double[] transform(double[][] r, double[] t, double[] p) {
        double[] v = multiply(r, p);
        for (int i = 0; i < 3; i++) v[i] += t[i];
        return v;
    }

    static double norm(double[] a) {
        double s = 0;
        for (double v : a) s += v * v;
        return Math.sqrt(s);
    }

    static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }

    /** Small pivoted linear solve used by the five-parameter LM fit. */
    static double[] solve(double[][] a, double[] b) {
        int n = b.length;
        double[][] m = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            System.arraycopy(a[i], 0, m[i], 0, n);
            m[i][n] = b[i];
        }
        for (int k = 0; k < n; k++) {
            int pivot = k;
            for (int i = k + 1; i < n; i++) if (Math.abs(m[i][k]) > Math.abs(m[pivot][k])) pivot = i;
            if (Math.abs(m[pivot][k]) < 1e-12) return null;
            double[] tmp = m[k]; m[k] = m[pivot]; m[pivot] = tmp;
            double divisor = m[k][k];
            for (int j = k; j <= n; j++) m[k][j] /= divisor;
            for (int i = 0; i < n; i++) if (i != k) {
                double f = m[i][k];
                for (int j = k; j <= n; j++) m[i][j] -= f * m[k][j];
            }
        }
        double[] x = new double[n];
        for (int i = 0; i < n; i++) x[i] = m[i][n];
        return x;
    }

    private HiveMath() { }
}
