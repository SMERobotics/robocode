package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.technodot.ftc.twentyfive.common.Artifact;
import com.technodot.ftc.twentyfive.common.ArtifactInventory;
import com.technodot.ftc.twentyfive.common.Controls;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DeviceIntake extends Device {
    public DcMotorEx motorIntake;
    public Servo servoLeft; // perspective of the robot
    public Servo servoRight; // perspective of the robot
    public RevColorSensorV3 colorLeft;
    public RevColorSensorV3 colorRight;

    public boolean leftPressed = false;
    public boolean rightPressed = false;
    public long leftActivated = 0;
    public long rightActivated = 0;
    public float rotationalOffset = 0.0f;

    public ArtifactInventory inventory = new ArtifactInventory();

    public final float LEFT_DISTANCE = 3.5f;
    public final float RIGHT_DISTANCE = 4.0f;

    // === Autonomous servo override ===
    private boolean servoOverride = false;
    private double overrideLeftPos = 0.3;  // default open for left
    private double overrideRightPos = 0.56; // default open for right

    @Override
    public void init(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        colorLeft = hardwareMap.get(RevColorSensorV3.class, "colorLeft");
        colorRight = hardwareMap.get(RevColorSensorV3.class, "colorRight");

        colorLeft.enableLed(true);
        colorRight.enableLed(true);
    }

    @Override
    public void start() {

    }

    @Override
    public void update(Gamepad gamepad) {
        if (Controls.intakeOut(gamepad)) {
            motorIntake.setPower(-1.0F);
        } else if (Controls.intakeIn(gamepad)) {
            motorIntake.setPower(1.0F);
        } else {
            motorIntake.setPower(0.0F);
        }

        long now = System.currentTimeMillis();

        boolean closeLeft = Controls.intakeServoLeft(gamepad) || Controls.intakeServoGreen(gamepad);
        if (closeLeft && !leftPressed) {
            leftActivated = now + 400;
            leftPressed = true;
        } else if (!closeLeft) {
            leftPressed = false;
        }

        boolean closeRight = Controls.intakeServoRight(gamepad) || Controls.intakeServoPurple(gamepad);
        if (closeRight && !rightPressed) {
            rightActivated = now + 400;
            rightPressed = true;
        } else if (!closeRight) {
            rightPressed = false;
        }

        rotationalOffset = 0.0f;

        if (now < leftActivated) {
            rotationalOffset = 5.06f;
        } else if (now < leftActivated + 100) {
            rotationalOffset = -5.06f;
        }

        if (now < rightActivated) {
            rotationalOffset = -11.77f;
        } else if (now < rightActivated + 100) {
            rotationalOffset = 11.77f;
        }

        // Servo control: respect autonomous override if enabled
        if (servoOverride) {
            try { servoLeft.setPosition(overrideLeftPos); } catch (Exception ignored) {}
            try { servoRight.setPosition(overrideRightPos); } catch (Exception ignored) {}
        } else {
            if (leftActivated - 250 < now && now < leftActivated) {
                servoLeft.setPosition(0.56); // left closed position
            } else {
                servoLeft.setPosition(0.3); // left open position
            }

            if (rightActivated - 250 < now && now < rightActivated) {
                servoRight.setPosition(0.3); // right closed position
            } else {
                servoRight.setPosition(0.56); // right open position
            }
        }

        // TODO: calibrate the color sensors to balls fr

        double leftDistance = colorLeft.getDistance(DistanceUnit.CM);
        if (leftDistance < LEFT_DISTANCE) {
            NormalizedRGBA leftColor = colorLeft.getNormalizedColors();
            inventory.setArtifact(ArtifactInventory.Side.LEFT, colorArtifact(leftColor));
        } else {
            inventory.setArtifact(ArtifactInventory.Side.LEFT, Artifact.NONE);
        }

        double rightDistance = colorRight.getDistance(DistanceUnit.CM);
        if (rightDistance < RIGHT_DISTANCE) {
            NormalizedRGBA rightColor = colorRight.getNormalizedColors();
            inventory.setArtifact(ArtifactInventory.Side.RIGHT, colorArtifact(rightColor));
        } else {
            inventory.setArtifact(ArtifactInventory.Side.RIGHT, Artifact.NONE);
        }
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("l_dist", colorLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("l_artifact", inventory.getArtifact(ArtifactInventory.Side.LEFT));

        telemetry.addData("r_dist", colorRight.getDistance(DistanceUnit.CM));
        telemetry.addData("r_artifact", inventory.getArtifact(ArtifactInventory.Side.RIGHT));
    }

    @Override
    public void stop() {

    }

    public ArtifactInventory getInventory() {
        return inventory;
    }

    public float getRotationalOffset() {
        return rotationalOffset;
    }

    private Artifact colorArtifact(NormalizedRGBA color) {
        float max = Math.max(color.red, Math.max(color.green, color.blue));
        float red = color.red / max;
        float green = color.green / max;
        float blue = color.blue / max;

        if (0.5 < red && red < 0.7 && 0.5 < green && green < 0.8 && blue > 0.99) {
            return Artifact.PURPLE;
        }
        if (0.2 < red && red < 0.4 && 0.6 < blue && blue < 0.9 && green > 0.99) {
            return Artifact.GREEN;
        }
        return Artifact.NONE;
    }

    // === Autonomous servo override API ===
    public void setServoOverride(boolean enabled) {
        this.servoOverride = enabled;
    }

    public boolean isServoOverride() {
        return servoOverride;
    }

    public void setServoPositions(double leftPos, double rightPos) {
        this.overrideLeftPos = leftPos;
        this.overrideRightPos = rightPos;
    }
}
