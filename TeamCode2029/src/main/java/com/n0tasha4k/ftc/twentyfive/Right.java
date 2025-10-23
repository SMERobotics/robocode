package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Right ")
public class Right extends LinearOpMode {

    private DcMotor right;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {

        right = hardwareMap.get(DcMotor.class, "right");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                right.setPower(-1);

                telemetry.addData("Right Pow", right.getPower());
                telemetry.update();
            }
        }
    }
}