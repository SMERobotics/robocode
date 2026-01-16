package org.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.technodot.ftc.twentyfive.common.Artifact;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.ArrayDeque;
import java.util.Deque;

public class DeviceIntake extends Device {

    public DcMotorEx motorIntake;

    public Servo servoLeft;
    public Servo servoRight;

    public RevColorSensorV3 colorLeft1;
    public RevColorSensorV3 colorLeft2;
    public RevColorSensorV3 colorRight1;
    public RevColorSensorV3 colorRight2;

    public static IntakeState intakeState = IntakeState.IDLE;
    public double intakeOverride;
    public double statusTelem; // just a graphable number for telemetry and timing purposes;

    private boolean leftTriggered;
    private boolean rightTriggered;
    private boolean leftActive;
    private boolean rightActive;
    private long leftActivationTime;
    private long rightActivationTime;

    private boolean sequenceTriggered;
    private boolean sequenceOverride;
    private boolean dualShortSequenceTriggered;
    public Deque<IntakeSide> sideDeque = new ArrayDeque<>();
    public long nextOptimizedTransfer; // timestamp for next optimized transfer attempt in ms
    public static IntakeSide targetSide = IntakeSide.LEFT;

    private boolean nudging;
    private boolean nudgeTriggered;
    private long nudgeStartTime;

    public volatile Artifact leftArtifact = Artifact.NONE;
    public volatile Artifact rightArtifact = Artifact.NONE;
    public Artifact queueArtifact = Artifact.PURPLE; // just a type

    private Thread colorSensorThread;
    private volatile boolean colorSensorThreadRunning = false;

    public enum IntakeState {
        IDLE,
        IN,
        OUT,
        OVERRIDE
    }

    public enum IntakeSide {
        LEFT,
        RIGHT,
        BOTH,
        NONE
    }

    public DeviceIntake(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        this.inputController = inputController;

        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");

        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        colorLeft1 = hardwareMap.get(RevColorSensorV3.class, "colorLeft1");
        colorLeft2 = hardwareMap.get(RevColorSensorV3.class, "colorLeft2");
        colorRight1 = hardwareMap.get(RevColorSensorV3.class, "colorRight1");
        colorRight2 = hardwareMap.get(RevColorSensorV3.class, "colorRight2");

        intakeState = IntakeState.IDLE;
    }

    @Override
    public void start() {
        startColorSensorThread();
    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;

//        this.updateColorSensors();

        // handle 2nd gamepad queue artifact selection (yours truly)
        if (ctrl.queuePurple()) {
            queueArtifact = Artifact.PURPLE;
        } else if (ctrl.queueGreen()) {
            queueArtifact = Artifact.GREEN;
        }

        // intake motor

        if (intakeState != IntakeState.OVERRIDE) {
            if (ctrl.intakeNudge() && !nudgeTriggered && intakeState == IntakeState.IDLE && !nudging) {
                // Start nudge: set target position to current + INTAKE_MOTOR_NUDGE_TICKS
                int targetPosition = motorIntake.getCurrentPosition() + Configuration.INTAKE_MOTOR_NUDGE_TICKS;
                motorIntake.setTargetPosition(targetPosition);
                motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorIntake.setPower(Configuration.INTAKE_MOTOR_NUDGE_POWER);
                nudging = true;
                nudgeTriggered = true;
                nudgeStartTime = System.currentTimeMillis();
            } else if (!ctrl.intakeNudge()) {
                nudgeTriggered = false;
            }
            
            if (ctrl.intakeOut()) {
                intakeState = IntakeState.OUT;
            } else if (ctrl.intakeIn()) {
                intakeState = IntakeState.IN;
            } else {
                intakeState = IntakeState.IDLE;
            }
        }

        // Cancel nudge if state is not IDLE
        if (intakeState != IntakeState.IDLE && nudging) {
            motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            nudging = false;
        }

        // Check if nudge is complete (with grace period to let motor start)
        long nudgeGracePeriod = 100; // ms
        if (nudging && (System.currentTimeMillis() - nudgeStartTime > nudgeGracePeriod) && !motorIntake.isBusy()) {
            motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorIntake.setPower(0);
            nudging = false;
        }

        switch (intakeState) {
            case IDLE:
                if (!nudging) {
                    motorIntake.setPower(0);
                }
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

        if (DeviceExtake.extakeState == DeviceExtake.ExtakeState.DUAL_SHORT) {
            if ((ctrl.sequenceShoot() || sequenceOverride) && !dualShortSequenceTriggered) {
                boolean hasLeft = leftArtifact != Artifact.NONE;
                boolean hasRight = rightArtifact != Artifact.NONE;

                if (hasLeft || hasRight) {
                    long now = System.currentTimeMillis();
                    if (hasLeft) {
                        leftActive = true;
                        leftActivationTime = now;
                    }
                    if (hasRight) {
                        rightActive = true;
                        rightActivationTime = now;
                    }
                    sideDeque.clear();
                    DeviceDrive.consumeExtakeFreeRotate();
                }

                sequenceOverride = false;
                dualShortSequenceTriggered = true;
            } else if (!ctrl.sequenceShoot()) {
                dualShortSequenceTriggered = false;
                sequenceTriggered = false;
            }
        } else {
            if ((ctrl.sequenceShoot() || sequenceOverride) && !sequenceTriggered) {
                boolean hasLeft = leftArtifact != Artifact.NONE;
                boolean hasRight = rightArtifact != Artifact.NONE;
                
                if (hasLeft && hasRight) {
                    // Both artifacts present: prioritize side matching queueArtifact
                    boolean leftMatchesQueue = leftArtifact == queueArtifact;
                    boolean rightMatchesQueue = rightArtifact == queueArtifact;

                    if (leftMatchesQueue && !rightMatchesQueue) {
                        // Left matches queue: launch left first
                        sideDeque.addLast(IntakeSide.LEFT);
                        sideDeque.addLast(IntakeSide.RIGHT);
                    } else if (rightMatchesQueue && !leftMatchesQueue) {
                        // Right matches queue: launch right first
                        sideDeque.addLast(IntakeSide.RIGHT);
                        sideDeque.addLast(IntakeSide.LEFT);
                    } else {
                        // Both or neither match: use alliance-based order
                        if (alliance == Alliance.BLUE) {
                            sideDeque.addLast(IntakeSide.LEFT);
                            sideDeque.addLast(IntakeSide.RIGHT);
                        } else {
                            sideDeque.addLast(IntakeSide.RIGHT);
                            sideDeque.addLast(IntakeSide.LEFT);
                        }
                    }
                } else if (hasLeft) {
                    // Only left artifact present
                    sideDeque.addLast(IntakeSide.LEFT);
                } else if (hasRight) {
                    // Only right artifact present
                    sideDeque.addLast(IntakeSide.RIGHT);
                }
                // If neither artifact is present, don't push anything

                sequenceOverride = false;
                nextOptimizedTransfer = System.currentTimeMillis();
                sequenceTriggered = true;
            } else if (!ctrl.sequenceShoot()) {
                sequenceTriggered = false;
            }
        }

        boolean shouldActivateLeft = shouldActivateLeft();
        boolean shouldActivateRight = shouldActivateRight();

        if (shouldActivateLeft && !leftTriggered) {
            leftActive = true;
            leftActivationTime = System.currentTimeMillis();
            leftTriggered = true;
            DeviceDrive.consumeExtakeFreeRotate();
        } else if (!shouldActivateLeft) {
            leftTriggered = false;
        }

        if (shouldActivateRight && !rightTriggered) {
            rightActive = true;
            rightActivationTime = System.currentTimeMillis();
            rightTriggered = true;
            DeviceDrive.consumeExtakeFreeRotate();
        } else if (!shouldActivateRight) {
            rightTriggered = false;
        }

        // self-note: should deactivate come after activate? or should activate have priority?
        boolean shouldDeactivateLeft = shouldDeactivateLeft();
        boolean shouldDeactivateRight = shouldDeactivateRight();

        if (shouldDeactivateLeft) leftActive = false;
        if (shouldDeactivateRight) rightActive = false;

        if (leftActive) {
            servoLeft.setPosition(Configuration.INTAKE_LEFT_ACTIVATION);
            DeviceExtake.unready();
        } else {
            servoLeft.setPosition(leftArtifact == Artifact.NONE ? Configuration.INTAKE_LEFT_DEACTIVATION : Configuration.INTAKE_LEFT_HOLD);
        }

        if (rightActive) {
            servoRight.setPosition(Configuration.INTAKE_RIGHT_ACTIVATION);
            DeviceExtake.unready();
        } else {
            servoRight.setPosition(rightArtifact == Artifact.NONE ? Configuration.INTAKE_RIGHT_DEACTIVATION : Configuration.INTAKE_RIGHT_HOLD);
        }

        statusTelem = (leftActive || rightActive) ? 670 : 0; // HEHEHEHA

        // Update targetSide prediction
        predictTargetSide();
    }

    /**
     * Updates targetSide to predict the next side the intake will shoot from.
     * This runs every update cycle, even when sideDeque is empty.
     */
    private void predictTargetSide() {
        // If there's a queued side, use that as the prediction
        if (!sideDeque.isEmpty()) {
            targetSide = sideDeque.peek();
            return;
        }

        // Otherwise, predict based on current state (same logic as queue population)
        boolean hasLeft = leftArtifact != Artifact.NONE;
        boolean hasRight = rightArtifact != Artifact.NONE;

        if (hasLeft && hasRight) {
            // Both artifacts present: prioritize side matching queueArtifact
            boolean leftMatchesQueue = leftArtifact == queueArtifact;
            boolean rightMatchesQueue = rightArtifact == queueArtifact;

            if (leftMatchesQueue && !rightMatchesQueue) {
                targetSide = IntakeSide.LEFT;
            } else if (rightMatchesQueue && !leftMatchesQueue) {
                targetSide = IntakeSide.RIGHT;
            } else {
                // Both or neither match: use alliance-based order
                targetSide = (alliance == Alliance.BLUE) ? IntakeSide.LEFT : IntakeSide.RIGHT;
            }
        } else if (hasLeft) {
            targetSide = IntakeSide.LEFT;
        } else if (hasRight) {
            targetSide = IntakeSide.RIGHT;
        } else {
            // No artifacts: predict based on alliance default
            targetSide = (alliance == Alliance.BLUE) ? IntakeSide.LEFT : IntakeSide.RIGHT;
        }
    }

    @Override
    public void stop() {
        stopColorSensorThread();
    }

    public void triggerSequenceShoot() {
        sequenceOverride = true;
    }

    public int getArtifactCount() {
        int count = 0;
        if (leftArtifact != Artifact.NONE) count++;
        if (rightArtifact != Artifact.NONE) count++;
        return count;
    }

    public boolean isEmpty() {
        return leftArtifact == Artifact.NONE && rightArtifact == Artifact.NONE;
    }

    public boolean isFull() {
        return leftArtifact != Artifact.NONE && rightArtifact != Artifact.NONE;
    }

    public void triggerNudge() {
        if (intakeState == IntakeState.IDLE && !nudging) {
            int targetPosition = motorIntake.getCurrentPosition() + Configuration.INTAKE_MOTOR_NUDGE_TICKS;
            motorIntake.setTargetPosition(targetPosition);
            motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorIntake.setPower(Configuration.INTAKE_MOTOR_NUDGE_POWER);
            nudging = true;
            nudgeStartTime = System.currentTimeMillis();
        }
    }

    public boolean isNudging() {
        return nudging;
    }

    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public void setIntakeIdle() {
        intakeState = IntakeState.IDLE;
        intakeOverride = 0;
    }

    public void setIntakeIn() {
        intakeState = IntakeState.OVERRIDE;
        intakeOverride = 1.0;
    }

    public void setIntakeOut() {
        intakeState = IntakeState.OVERRIDE;
        intakeOverride = -1.0;
    }

    public void setIntakeOverride() {
        intakeState = IntakeState.OVERRIDE;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void updateColorSensors() {
        // color sensors

        // TODO: implement color cacheing
        // HOWEVER, one color sensor (i forgot) is exhibiting a lot of false positives
        // this would fuck cacheing over. if it works, don't touch it

//        if (isArtifactLeft(colorLeft1.getDistance(DistanceUnit.CM), colorLeft2.getDistance(DistanceUnit.CM))) {
//            NormalizedRGBA cl1_n = colorLeft1.getNormalizedColors();
//            NormalizedRGBA cl2_n = colorLeft2.getNormalizedColors();
//            Artifact cl1_a = getArtifactColor(cl1_n);
//            Artifact cl2_a = getArtifactColor(cl2_n);
//            leftArtifact = combineArtifactColors(cl1_a, cl2_a);
//        } else {
//            leftArtifact = Artifact.NONE;
//        }
//
//        if (isArtifactRight(colorRight1.getDistance(DistanceUnit.CM), colorRight2.getDistance(DistanceUnit.CM))) {
//            NormalizedRGBA cr1_n = colorRight1.getNormalizedColors();
//            NormalizedRGBA cr2_n = colorRight2.getNormalizedColors();
//            Artifact cr1_a = getArtifactColor(cr1_n);
//            Artifact cr2_a = getArtifactColor(cr2_n);
//            rightArtifact = combineArtifactColors(cr1_a, cr2_a);
//        } else {
//            rightArtifact = Artifact.NONE;
//        }

        // holy oneliners
        leftArtifact = isArtifactLeft(colorLeft1.getDistance(DistanceUnit.CM), colorLeft2.getDistance(DistanceUnit.CM)) ? combineArtifactColors(getArtifactColor(colorLeft1.getNormalizedColors()), getArtifactColor(colorLeft2.getNormalizedColors())) : Artifact.NONE;
        rightArtifact = isArtifactRight(colorRight1.getDistance(DistanceUnit.CM), colorRight2.getDistance(DistanceUnit.CM)) ? combineArtifactColors(getArtifactColor(colorRight1.getNormalizedColors()), getArtifactColor(colorRight2.getNormalizedColors())) : Artifact.NONE;
    }

    private void startColorSensorThread() {
        if (colorSensorThread != null && colorSensorThread.isAlive()) {
            return;
        }

        colorSensorThreadRunning = true;
        colorSensorThread = new Thread(() -> {
            while (colorSensorThreadRunning) {
                try {
                    updateColorSensors();
                    Thread.sleep(50);

                } catch (Exception e) {
                    // chill out
                }
            }
        }, "UpdateColorSensors");
        colorSensorThread.setDaemon(true);
        colorSensorThread.start();
    }

    private void stopColorSensorThread() {
        colorSensorThreadRunning = false;
        if (colorSensorThread != null) {
            try {
                colorSensorThread.join(100); // be patient for 100ms
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            colorSensorThread = null;
        }
    }

    private boolean shouldActivateLeft() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;

        // Manual control
        if (ctrl.intakeServoLeft()) return true;

        // Automatic sequence control
        if (DeviceExtake.extakeState == DeviceExtake.ExtakeState.DUAL_SHORT) return false;
        if (!sideDeque.isEmpty()) {
            IntakeSide nextSide = sideDeque.peek();
            if (nextSide == IntakeSide.LEFT && System.currentTimeMillis() >= nextOptimizedTransfer) {
                sideDeque.poll();
                nextOptimizedTransfer = System.currentTimeMillis() + (DeviceExtake.extakeState == DeviceExtake.ExtakeState.SHORT ? Configuration.INTAKE_SERVO_SHORT_DELAY_MS : Configuration.INTAKE_SERVO_LONG_DELAY_MS);
                return true;
            }
        }

        return false;
    }

    private boolean shouldActivateRight() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;

        // Manual control
        if (ctrl.intakeServoRight()) return true;

        // Automatic sequence control
        if (DeviceExtake.extakeState == DeviceExtake.ExtakeState.DUAL_SHORT) return false;
        if (!sideDeque.isEmpty()) {
            IntakeSide nextSide = sideDeque.peek();
            if (nextSide == IntakeSide.RIGHT && System.currentTimeMillis() >= nextOptimizedTransfer) {
                sideDeque.poll();
                nextOptimizedTransfer = System.currentTimeMillis() + (DeviceExtake.extakeState == DeviceExtake.ExtakeState.SHORT ? Configuration.INTAKE_SERVO_SHORT_DELAY_MS : Configuration.INTAKE_SERVO_LONG_DELAY_MS);
                return true;
            }
        }

        return false;
    }

    private boolean shouldDeactivateLeft() {
        return System.currentTimeMillis() - leftActivationTime > Configuration.INTAKE_SERVO_INTERVAL_MS;
    }

    private boolean shouldDeactivateRight() {
        return System.currentTimeMillis() - rightActivationTime > Configuration.INTAKE_SERVO_INTERVAL_MS;
    }

    public static boolean isArtifactLeft(double cm1, double cm2) {
        return (cm1 <= 3.8) && (cm2 <= 4);
    }

    public static boolean isArtifactRight(double cm1, double cm2) {
        return (cm1 <= 3.35) || (cm2 <= 2.17); // it was actually the right servo mb
    }

    public static Artifact getArtifactColor(NormalizedRGBA color) {
        // TODO: rewrite to use hue instead of generic RGB comparisons
        // if its not broken don't touch it
        // and it aint broke (rn)
        if (!(color.red >= 0.002 || color.green >= 0.002 || color.blue >= 0.002)) return Artifact.NONE;
        if (color.blue >= color.green) return Artifact.PURPLE;
        if (color.red > color.green) return Artifact.NONE; // orange ramp should trigger this, TEST!!! (it doesn't but its also not broken, so i'll keep it)
        if (color.blue < color.green) return Artifact.GREEN;
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
