package com.technodot.ftc.twentyfour.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfour.Configuration;

public class DeviceDrive extends Device {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public Drivetrain drivetrain;
    public Drivetype drivetype;

    public float speedMultiplier = 1.0F;

    public DeviceDrive(HardwareMap hardwareMap, Drivetrain drivetrainType, Drivetype drivetypeType) {
        drivetrain = drivetrainType;
        drivetype = drivetypeType;

        switch (drivetrain) {
            case TANK:
                frontLeft = hardwareMap.get(DcMotor.class, "driveLeft");
                frontRight = hardwareMap.get(DcMotor.class, "driveRight");
                break;
            case MECANUM:
            case FOURWHEEL:
                frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
                frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
                backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
                backRight = hardwareMap.get(DcMotor.class, "driveBackRight");
                break;
        }
    }

    public void setDrive(float powerFrontLeft, float powerFrontRight, float powerBackLeft, float powerBackRight) {
        if (drivetrain != Drivetrain.TANK) {
            frontLeft.setPower(powerFrontLeft);
            frontRight.setPower(powerFrontRight);
            backLeft.setPower(powerBackLeft);
            backRight.setPower(powerBackRight);
        }
    }

    public void setDrive(float powerFrontLeft, float powerFrontRight) {
        if (drivetrain == Drivetrain.TANK) {
            frontLeft.setPower(-powerFrontLeft);
            frontRight.setPower(powerFrontRight);
        }
    }

    public void runDrive(float powerFrontLeft, float powerFrontRight, float powerBackLeft, float powerBackRight, float seconds) {
        long current = System.currentTimeMillis();
        while (System.currentTimeMillis() < current + (long) (seconds * 1000)) {
            setDrive(powerFrontLeft, powerFrontRight, powerBackLeft, powerBackRight);
        }
    }

    public void runDrive(float powerFrontLeft, float powerFrontRight, float seconds) {
        long current = System.currentTimeMillis();
        while (System.currentTimeMillis() < current + (long) (seconds * 1000)) {
            setDrive(powerFrontLeft, powerFrontRight);
        }
    }

    public void updatePreciseMode(boolean preciseMode) {
        if (preciseMode) {
            speedMultiplier = Configuration.drivePreciseModeMultiplier;
        } else {
            speedMultiplier = 1.0F;
        }
    }

    @SuppressWarnings("ConstantConditions")
    public void updateDrive(float leftX, float leftY, float rightX, float rightY) {
        float powerFrontLeft = 0.0F;
        float powerFrontRight = 0.0F;
        float powerBackLeft = 0.0F;
        float powerBackRight = 0.0F;

        if (drivetrain != Drivetrain.MECANUM) {
            float calculateLeft = 0.0F;
            float calculateRight = 0.0F;

            switch (drivetype) {
                case TANK:
                    if (Configuration.driveTurnDirection == 1) {
                        calculateLeft = leftY;
                        calculateRight = rightY;
                    } else if (Configuration.driveTurnDirection == -1) {
                        calculateLeft = rightY;
                        calculateRight = leftY;
                    }
                    break;
                case ARCADE:
                    leftX *= Configuration.driveTurnDirection;

                    float maximum = Math.max(Math.abs(leftX), Math.abs(leftY));
                    float total = leftX + leftY;
                    float difference = leftY - leftX;

                    if (leftY >= 0) {
                        if (leftX >= 0) {
                            calculateLeft = maximum;
                            calculateRight = difference;
                        } else {
                            calculateLeft = total;
                            calculateRight = maximum;
                        }
                    } else {
                        if (leftX >= 0) {
                            calculateLeft = total;
                            calculateRight = -maximum;
                        } else {
                            calculateLeft = -maximum;
                            calculateRight = difference;
                        }
                    }
                    break;
                case TECHNODRIVE_V1:
                    // TODO: test and rework
                    float heading = (float) (Math.atan(rightX / rightY) * 0.636619772386F);
                    if (rightX < 0) {
                        heading *= -1;
                    }
                    if (rightY < 0) {
                        heading += 2;
                    }
                    heading -= 1;
                    heading *= -1;
                    if (Configuration.driveTurnDirection == 1) {
                        if (rightX < 0) {
                            calculateLeft *= heading;
                        } else if (rightX > 0) {
                            calculateRight *= -heading;
                        }
                    } else if (Configuration.driveTurnDirection == -1) {
                        if (rightX < 0) {
                            calculateRight *= heading;
                        } else if (rightX > 0) {
                            calculateLeft *= -heading;
                        }
                    }
                    break;
            }

            switch (drivetrain) {
                case TANK:
                    powerFrontLeft = -calculateLeft;
                    powerFrontRight = calculateRight;
                    break;
                case FOURWHEEL:
                    powerFrontLeft = calculateLeft;
                    powerBackLeft = calculateLeft;
                    powerFrontRight = calculateRight;
                    powerBackRight = calculateRight;
                    break;
            }

            float totalLeft = speedMultiplier * Configuration.driveDirection * Configuration.driveLeftMultiplier;
            float totalRight = speedMultiplier * Configuration.driveDirection * Configuration.driveRightMultiplier;

            powerFrontLeft *= totalLeft;
            powerBackLeft *= totalLeft;
            powerFrontRight *= totalRight;
            powerBackRight *= totalRight;

            frontLeft.setPower(powerFrontLeft);
            frontRight.setPower(powerFrontRight);
            if (drivetrain != Drivetrain.TANK) {
                backLeft.setPower(powerBackLeft);
                backRight.setPower(powerBackRight);
            }
        } else {
            // Drivetrain.MECANUM has override controls for omnidirectionality

            leftY *= Configuration.driveDirection;
            rightX *= Configuration.driveTurnDirection;

            powerFrontLeft = leftY - leftX - rightX;
            powerFrontRight = leftY + leftX + rightX;
            powerBackLeft = -(leftY + leftX - rightX);
            powerBackRight = -(leftY - leftX + rightX);

            powerFrontLeft *= speedMultiplier;
            powerFrontRight *= speedMultiplier;
            powerBackLeft *= speedMultiplier;
            powerBackRight *= speedMultiplier;

            frontLeft.setPower(powerFrontLeft);
            frontRight.setPower(powerFrontRight);
            backLeft.setPower(powerBackLeft);
            backRight.setPower(powerBackRight);
        }

    }
}
