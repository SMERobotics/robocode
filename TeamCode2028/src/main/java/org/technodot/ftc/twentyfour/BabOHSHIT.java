package org.technodot.ftc.twentyfour;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="BaboBotz", group="TechnoCode")
public class BabOHSHIT extends LinearOpMode {

    private DcMotor driveRight;
    private DcMotor driveLeft;
    private DcMotor slideMain;
    private DcMotor clawArm;
    private Servo clawMain;

    @Override
    public void runOpMode() {
        driveRight = hardwareMap.get(DcMotor.class, "driveRight");
        driveLeft = hardwareMap.get(DcMotor.class, "driveLeft");
        slideMain = hardwareMap.get(DcMotor.class, "slideMain");
        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        clawMain = hardwareMap.get(Servo.class, "clawMain");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driveLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x));
                driveRight.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x));

                if (gamepad1.left_bumper) {
                    slideMain.setPower(-1);
                } else if (gamepad1.right_bumper) {
                    slideMain.setPower(1);
                } else {
                    slideMain.setPower(0);
                }

                if (gamepad1.y) {
                    clawArm.setPower(0.4);
                } else if (gamepad1.x) {
                    clawArm.setPower(-0.4);
                } else {
                    clawArm.setPower(0);
                }

                if (gamepad1.a) {
                    clawMain.setPosition(0);
                } else if (gamepad1.b) {
                    clawMain.setPosition(1);
                }

                telemetry.update();
            }
        }
    }
}