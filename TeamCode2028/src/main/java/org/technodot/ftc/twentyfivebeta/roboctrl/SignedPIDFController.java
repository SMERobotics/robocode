package org.technodot.ftc.twentyfivebeta.roboctrl;

public class SignedPIDFController extends PIDFController {
    public SignedPIDFController(double kp, double ki, double kd, double kf) {
        super(kp, ki, kd, kf);
    }

    public SignedPIDFController(double kp, double ki, double kd, double kf, double sp, double pv) {
        super(kp, ki, kd, kf, sp, pv);
    }

    @Override
    public double calculate(double pv) {
        double sign = 0;
        if (pv > 0) sign = -1;
        if (pv < 0) sign = 1;
        return super.calculate(pv) + sign * super.getF();
    }
}
