package org.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

public class DeviceExtake extends Device {

    public DcMotorEx motorExtakeLeft;
    public DcMotorEx motorExtakeRight;
//    public PIDFController extakeLeftPIDF;
//    public PIDFController extakeRightPIDF;

    boolean prevExtakeClose;
    boolean prevExtakeFar;

    public ExtakeState extakeState = ExtakeState.IDLE;
    public double targetVelocity; // current vel setpoint
    public double extakeOverride;
    public static int stabilizationCycles;
    public boolean rumbled;

    public enum ExtakeState {
        IDLE,
        SHORT,
        LONG,
        REVERSE,
        ZERO,
        OVERRIDE
    }

    public DeviceExtake(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        this.inputController = inputController;

        motorExtakeLeft = hardwareMap.get(DcMotorEx.class, "motorExtakeLeft");
        motorExtakeRight = hardwareMap.get(DcMotorEx.class, "motorExtakeRight");

        motorExtakeLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorExtakeRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorExtakeLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorExtakeRight.setDirection(DcMotorEx.Direction.REVERSE);

        motorExtakeLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorExtakeRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

//        extakeLeftPIDF = new PIDFController(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);
//        extakeRightPIDF = new PIDFController(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);

        motorExtakeLeft.setVelocityPIDFCoefficients(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);
        motorExtakeRight.setVelocityPIDFCoefficients(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);

        extakeState = ExtakeState.IDLE;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;

        boolean extakeFar = ctrl.extakeFar();
        boolean extakeClose = ctrl.extakeClose();

        if (ctrl.extakeReverse()) {
            extakeState = ExtakeState.REVERSE;
        } else if (extakeFar && !prevExtakeFar) {
            extakeState = extakeState == ExtakeState.LONG ? ExtakeState.IDLE : ExtakeState.LONG;
            stabilizationCycles = 0;
        } else if (extakeClose && !prevExtakeClose) {
            extakeState = extakeState == ExtakeState.SHORT ? ExtakeState.IDLE : ExtakeState.SHORT;
            stabilizationCycles = 0;
        } else if (extakeState == ExtakeState.REVERSE) {
            extakeState = ExtakeState.ZERO;
        }

        prevExtakeFar = extakeFar;
        prevExtakeClose = extakeClose;

        // PIDF coefficients should be applied only at init
        // temp moved to update cycle so Configuration changes will take effect
//        motorExtakeLeft.setVelocityPIDFCoefficients(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);
//        motorExtakeRight.setVelocityPIDFCoefficients(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);

//        extakeLeftPIDF.setPIDF(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);
//        extakeRightPIDF.setPIDF(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);

        switch (extakeState) {
            case IDLE:
                motorExtakeLeft.setPower(0.0);
                motorExtakeRight.setPower(0.0);
                targetVelocity = 0;
                break;
            case SHORT:
                setTargetVelocity(Configuration.EXTAKE_MOTOR_SPEED_SHORT);
                break;
            case LONG:
                setTargetVelocity(Configuration.EXTAKE_MOTOR_SPEED_LONG);
                break;
            case REVERSE:
                motorExtakeLeft.setPower(-1.0);
                motorExtakeRight.setPower(-1.0);
                targetVelocity = 0;
                break;
            case ZERO:
                setTargetVelocity(0.0);
                break;
            case OVERRIDE:
                setTargetVelocity(extakeOverride);
                break;
        }

        if (isReady() && !rumbled) {
            ctrl.vibrateExtakeReady();
            rumbled = true;
        } else if (!isReady()) {
            rumbled = false;
        }
    }

    @Override
    public void stop() {

    }

    /**
     * Set the extake state. Changes will take effect next motor update.
     * @param extakeState The ExtakeState to set.
     */
    public void setExtakeState(ExtakeState extakeState) {
        this.extakeState = extakeState;
    }

    /**
     * Get the current extake state.
     * @return Current extake state.
     */
    public ExtakeState getExtakeState() {
        return this.extakeState;
    }

    /**
     * Set the extake override velocity. Velocity is only applied when DeviceExtake.currentState == ExtakeState.OVERRIDE.
     * @param extakeOverride The velocity in default units.
     */
    public void setExtakeOverride(double extakeOverride) {
        this.extakeOverride = extakeOverride;
    }

    /**
     * Get the extake override velocity.
     * @return The velocity in default units.
     */
    public double getExtakeOverride() {
        return this.extakeOverride;
    }

    /**
     * Check if the extake motors are physically idle (not spinning).
     * @return If the extake motors are idle.
     */
    public boolean isIdle() {
        return Math.abs(this.motorExtakeLeft.getVelocity()) <= 20 && Math.abs(this.motorExtakeRight.getVelocity()) <= 20;
    }

    public boolean isReady() {
        switch (extakeState) {
            case SHORT:
            case LONG:
                stabilizationCycles = Math.abs(motorExtakeLeft.getVelocity() - targetVelocity) <= Configuration.EXTAKE_MOTOR_SPEED_TOLERANCE && Math.abs(motorExtakeRight.getVelocity() - targetVelocity) <= Configuration.EXTAKE_MOTOR_SPEED_TOLERANCE ? stabilizationCycles + 1 : stabilizationCycles;
                break;
            default:
                stabilizationCycles = 0;
                return false;
        }
        return stabilizationCycles >= Configuration.EXTAKE_STABILIZATION_CYCLES;
    }

    public static void unready() {
        stabilizationCycles = 0;
    }

    /**
     * Set the velocity of both extake motors.
     * @param targetVelocity The velocity in default units.
     */
    private void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;

        // SUPER FEEDFORWARD 💪💪💪
        double leftVelocity = motorExtakeLeft.getVelocity();
        if (Math.abs(targetVelocity - leftVelocity) > Configuration.EXTAKE_MOTOR_SUPER_FEEDFORWARD_THRESHOLD) {
            motorExtakeLeft.setPower(Math.signum(targetVelocity - leftVelocity));
        } else {
            motorExtakeLeft.setVelocity(targetVelocity);
        }

        double rightVelocity = motorExtakeRight.getVelocity();
        if (Math.abs(targetVelocity - rightVelocity) > Configuration.EXTAKE_MOTOR_SUPER_FEEDFORWARD_THRESHOLD) {
            motorExtakeRight.setPower(Math.signum(targetVelocity - rightVelocity));
        } else {
            motorExtakeRight.setVelocity(targetVelocity);
        }

//        motorExtakeLeft.setVelocity(targetVelocity);
//        motorExtakeRight.setVelocity(targetVelocity);
//        motorExtakeLeft.setPower(extakeLeftPIDF.calculate(motorExtakeLeft.getVelocity(), extakeVelocity));
//        motorExtakeRight.setPower(extakeRightPIDF.calculate(motorExtakeRight.getVelocity(), extakeVelocity));
    }
}
