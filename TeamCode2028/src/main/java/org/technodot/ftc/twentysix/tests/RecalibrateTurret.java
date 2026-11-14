package org.technodot.ftc.twentysix.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="RecalibrateTurret", group="TechnoCode")
public class RecalibrateTurret extends OpMode {

    private DcMotorEx motorTurret;

    @Override
    public void init() {
        motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");
        motorTurret.setPower(0);
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        showPosition();
    }

    @Override
    public void init_loop() {
        showPosition();
    }

    @Override
    public void loop() {
        showPosition();
    }

    @Override
    public void stop() {
        if (motorTurret != null) {
            motorTurret.setPower(0);
            motorTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorTurret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void showPosition() {
        telemetry.addData("turret_pos", motorTurret.getCurrentPosition());
        telemetry.update();
    }
}
