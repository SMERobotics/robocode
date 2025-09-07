package com.technodot.ftc.twentyfour.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfour.Configuration;

public class DeviceSlide extends Device {
    public DcMotor slideMain;

    private int slideStatus = 0;
    private long slideTime = 0;
    private long slideTimer = 0;
    public float speedMultiplier = 1.0F;

    public DeviceSlide(HardwareMap hardwareMap) {
        slideMain = hardwareMap.get(DcMotor.class, "slideMain");
        slideMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void updatePreciseMode(boolean preciseMode) {
        if (preciseMode) {
            speedMultiplier = Configuration.slidePreciseModeMultiplier;
        } else {
            speedMultiplier = 1.0F;
        }
    }

    public void resetSlideEncoder() {
        slideMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @SuppressWarnings("ConstantConditions")
    public void updateSlide(boolean slideExtend, boolean slideRetract, boolean slideExtendForce, boolean slideRetractForce) {
        long currentTime = System.currentTimeMillis();

        if (slideExtend) { // extend slide button pressed
            if (slideStatus == 0) { // slide not moving
                if (slideTime < Configuration.slideConstant) { // slide can move
                    slideMain.setPower(Configuration.slideDirection * Configuration.slideSpeedMultiplier);
                    slideTimer = currentTime;
                    slideStatus = 1;
                }
            } else { // slide is moving
                if (currentTime >= slideTimer - slideTime + Configuration.slideConstant) { // slide can't move
                    slideMain.setPower(0.0);
                    slideTime = Configuration.slideConstant;
                    slideStatus = 0;
                    slideTimer = 0;
                } else if (slideStatus != 0) { // slide is moving
                    slideMain.setPower(Configuration.slideDirection * Configuration.slideSpeedMultiplier);
                }
            }
        } else if (slideRetract) { // retract slide button pressed
            if (slideStatus == 0) { // slide not moving
                if (slideTime > 0) { // slide can move
                    slideMain.setPower(-1.0 * Configuration.slideDirection * Configuration.slideSpeedMultiplier * Configuration.slideRetractMultiplier);
                    slideTimer = currentTime;
                    slideStatus = -1;
                }
            } else { // slide is moving
                if (currentTime >= slideTimer + slideTime) { // slide can't move
                    slideMain.setPower(0.0);
                    slideTime = 0;
                    slideStatus = 0;
                    slideTimer = 0;
                } else if (slideStatus != 0) { // slide is moving
                    slideMain.setPower(-1.0 * Configuration.slideDirection * Configuration.slideSpeedMultiplier * Configuration.slideRetractMultiplier);
                }
            }
        } else { // neither buttons pressed
            slideMain.setPower(0.0);
            if (slideStatus == 1) {
                slideTime += currentTime - slideTimer;
                slideStatus = 0;
                slideTimer = 0;
            } else if (slideStatus == -1) {
                slideTime -= currentTime - slideTimer;
                slideStatus = 0;
                slideTimer = 0;
            }
        }
    }

    public void updateSlideForce(boolean slideExtend, boolean slideRetract, boolean slideExtendForce, boolean slideRetractForce) {
        if (slideExtendForce) {
            slideMain.setPower(Configuration.slideDirection * Configuration.slideSpeedMultiplier);
        } else if (slideRetractForce) {
            slideMain.setPower(-1.0 * Configuration.slideDirection * Configuration.slideSpeedMultiplier * Configuration.slideRetractMultiplier);
        } else if (!(slideExtend || slideRetract)) {
            slideMain.setPower(0.0);
        }
    }
}
