package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BADdieAUTONOMOUS (Blocks to Java)")
public class BADdieAUTONOUMOUS extends LinearOpMode {

    private DcMotor left;
    private DcMotor index;
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
        left = hardwareMap.get(DcMotor.class, "left");
        index = hardwareMap.get(DcMotor.class, "index");
        right = hardwareMap.get(DcMotor.class, "right");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                left.setPower(1/1.07);
                right.setPower(1);
                sleep(1000);
                left.setPower(0);
                right.setPower(0);
                telemetry.update();
            }
        }
    }
}