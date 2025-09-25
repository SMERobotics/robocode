package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.util.Toggler;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Miles' Android Studio Revision")
public class testtele extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "activeShooter");
        DcMotor index = hardwareMap.dcMotor.get("index");

        Servo finger = hardwareMap.servo.get("indexfinger");

        Toggler shootertoggle = new Toggler();

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        left.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            shooter.setPower(1);
            float gamepadX;
            float gamepadY;

            if (gamepad1.a) {
                finger.setPosition(1);
                index.setPower(-1);
            } else {
                finger.setPosition(0);
                index.setPower(0);
            }

            if (gamepad1.y) {
                shooter.setVelocity(2000);
            } else {
                shooter.setVelocity(0);
            }

            telemetry.addData("Left Pow", left.getPower());
            telemetry.addData("Right Pow", right.getPower());


            gamepadX = -gamepad1.right_stick_x;
            gamepadY = -gamepad1.left_stick_y;
            left.setPower(gamepadY - gamepadX);
            right.setPower(gamepadY + gamepadX);
        }

    }
}
