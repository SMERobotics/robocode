package com.mech.ftc.twentyfive.defaults;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class IndexPID extends OpMode {
    private PIDController controller;

    public static double p = 0.01, i = 0.01, d = 0.000277;
    public static double f = 0.01;


    public static int targetPosition = 0;

    private DcMotorEx indexMotor;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        indexMotor = hardwareMap.get(DcMotorEx.class, "indexMotor");

    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int indexPosition = indexMotor.getCurrentPosition();
        double pid = controller.calculate(indexPosition, targetPosition);
        double TICKS_PER_REV = 28 * 27;
        double ff = Math.cos(Math.toRadians(targetPosition / TICKS_PER_REV * 360)) * f;

        double power = pid + ff;

        indexMotor.setPower(power);

        telemetry.addData("Index Position", indexPosition);
        telemetry.addData("Target Position", targetPosition);
        telemetry.update();



    }
}
