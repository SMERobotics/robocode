package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeviceDrive extends Device {

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    public float speedMultiplier = 1.0F;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // toggle all of them to change robot drive direction
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void update(Gamepad gamepad) {
        // FTC gamepad: up on stick is negative, so invert Y for forward
        float forward = -gamepad.left_stick_y;   // intended forward/back motion
        float lateral = gamepad.left_stick_x;    // intended left/right strafe
        float turn = gamepad.right_stick_x;      // rotation
        // Hardware wiring/frame is rotated 90° so swap mapping: what produces physical forward is current strafe pattern
        // Map joystick forward -> strafe term, joystick lateral -> drive term
        update(lateral, turn, forward); // (drive, turn, strafe)
    }

    /**
     * Update mecanum drive motor powers.
     * @param drive forward/backward component (-1..1 forward positive)
     * @param turn rotational component (-1..1 left negative/right positive depending on convention)
     * @param strafe lateral component (-1..1 right positive)
     */
    public void update(float drive, float turn, float strafe) {
        // Standard mecanum formula (robot-centric)
        float frontLeft = drive + strafe + turn;
        float frontRight = drive - strafe - turn;
        float backLeft = drive - strafe + turn;
        float backRight = drive + strafe - turn;

        // Normalize so no value exceeds magnitude 1
        float max = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight))
        );
        if (max > 1.0f) {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }

        // Apply speed multiplier then clip to [-1,1]
        motorFrontLeft.setPower(clip(frontLeft * speedMultiplier));
        motorFrontRight.setPower(clip(frontRight * speedMultiplier));
        motorBackLeft.setPower(clip(backLeft * speedMultiplier));
        motorBackRight.setPower(clip(backRight * speedMultiplier));
    }

    private float clip(float p) {
        if (p > 1f) return 1f;
        if (p < -1f) return -1f;
        return p;
    }

    public void zero() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
}
