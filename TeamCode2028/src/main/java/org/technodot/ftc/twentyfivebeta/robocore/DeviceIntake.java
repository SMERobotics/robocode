package org.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.technodot.ftc.twentyfive.common.Artifact;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

public class DeviceIntake extends Device {

    public DcMotorEx motorIntake;

    public Servo servoLeft;
    public Servo servoRight;

    public RevColorSensorV3 colorLeft1;
    public RevColorSensorV3 colorLeft2;
    public RevColorSensorV3 colorRight1;
    public RevColorSensorV3 colorRight2;

    public IntakeState intakeState = IntakeState.IDLE;
    public double intakeOverride;

    private boolean leftTriggered;
    private boolean rightTriggered;
    private boolean leftActive;
    private boolean rightActive;
    private long leftActivationTime;
    private long rightActivationTime;

    public enum IntakeState {
        IDLE,
        IN,
        OUT,
        OVERRIDE
    }

    public DeviceIntake(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        this.inputController = inputController;

        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        colorLeft1 = hardwareMap.get(RevColorSensorV3.class, "colorLeft1");
        colorLeft2 = hardwareMap.get(RevColorSensorV3.class, "colorLeft2");
        colorRight1 = hardwareMap.get(RevColorSensorV3.class, "colorRight1");
        colorRight2 = hardwareMap.get(RevColorSensorV3.class, "colorRight2");

        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeState = IntakeState.IDLE;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;

        // intake motor

        if (intakeState != IntakeState.OVERRIDE) {
            if (ctrl.intakeOut()) {
                intakeState = IntakeState.OUT;
            } else if (ctrl.intakeIn()) {
                intakeState = IntakeState.IN;
            } else {
                intakeState = IntakeState.IDLE;
            }
        }

        switch (intakeState) {
            case IDLE:
                motorIntake.setPower(0);
                break;
            case IN:
                motorIntake.setPower(1.0);
                break;
            case OUT:
                motorIntake.setPower(-1.0);
                break;
            case OVERRIDE:
                motorIntake.setPower(intakeOverride);
                break;
        }

        // intake servo ctrl (L & R)

        boolean shouldActivateLeft = shouldActivateLeft();
        boolean shouldActivateRight = shouldActivateRight();

        if (shouldActivateLeft && !leftTriggered) {
            leftActive = true;
            leftTriggered = true;
            leftActivationTime = System.currentTimeMillis();
        } else if (!shouldActivateLeft) {
            leftTriggered = false;
        }

        if (shouldActivateRight && !rightTriggered) {
            rightActive = true;
            rightTriggered = true;
            rightActivationTime = System.currentTimeMillis();
        } else if (!shouldActivateRight) {
            rightTriggered = false;
        }

        // should deactivate come after activate? or should activate have priority?
        boolean shouldDeactivateLeft = shouldDeactivateLeft();
        boolean shouldDeactivateRight = shouldDeactivateRight();

        if (shouldDeactivateLeft) leftActive = false;
        if (shouldDeactivateRight) rightActive = false;

        if (leftActive) {
            servoLeft.setPosition(Configuration.INTAKE_LEFT_ACTIVATION);
        } else {
            servoLeft.setPosition(Configuration.INTAKE_LEFT_DEACTIVATION);
        }

        if (rightActive) {
            servoRight.setPosition(Configuration.INTAKE_RIGHT_ACTIVATION);
        } else {
            servoRight.setPosition(Configuration.INTAKE_RIGHT_DEACTIVATION);
        }
    }

    @Override
    public void stop() {

    }

    private boolean shouldActivateLeft() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;
        return ctrl.intakeServoLeft();
    }

    private boolean shouldActivateRight() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;
        return ctrl.intakeServoRight();
    }

    private boolean shouldDeactivateLeft() {
        return System.currentTimeMillis() - leftActivationTime > Configuration.INTAKE_SERVO_INTERVAL_MS;
    }

    private boolean shouldDeactivateRight() {
        return System.currentTimeMillis() - rightActivationTime > Configuration.INTAKE_SERVO_INTERVAL_MS;
    }

    public static Artifact getArtifactColor(NormalizedRGBA color, double distanceCM) {
        // TODO: ask ts gpt-5.2 to write ts algorithm
        return Artifact.NONE;
    }

    public static Artifact combineArtifactColors(Artifact a, Artifact b) {
        // PURPLE over everything.
        // GREEN over NONE.
        // NONE only with NONE.

        if (a == Artifact.PURPLE || b == Artifact.PURPLE) {
            return Artifact.PURPLE;
        } else if (a == Artifact.GREEN || b == Artifact.GREEN) {
            return Artifact.GREEN;
        } else {
            return Artifact.NONE;
        }
    }
}
