package com.shark.ftc.twentysix;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "C++ Hello World", group = "Test")
public class HelloNativeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Press START to call C++.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        String message = new HelloBridge().getHelloWorldMessage();
        while (opModeIsActive()) {
            telemetry.addData("Native message", message);
            telemetry.update();
            sleep(100);
        }
    }
}
