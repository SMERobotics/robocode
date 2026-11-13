package org.technodot.ftc.twentysix.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;

public class DeviceIntake extends Device {

    public static final double TRANSFER_OPEN_POS = 0.04;
    public static final double TRANSFER_CLOSED_POS = 0.18;

    public DcMotorEx motorIntakeLeft;
    public DcMotorEx motorIntakeRight;
    public Servo servoTransfer;

    public double transferPos = 0.0;

    public DeviceIntake(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, GamepadBindings gamepad, Telemetry telemetry) {
        super.init(hardwareMap, gamepad, telemetry);

        motorIntakeLeft = hardwareMap.get(DcMotorEx.class, "motorIntakeLeft");
        motorIntakeRight = hardwareMap.get(DcMotorEx.class, "motorIntakeRight");
        motorIntakeLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorIntakeRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorIntakeLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorIntakeRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        servoTransfer = hardwareMap.get(Servo.class, "servoTransfer");
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        double power = gamepad.intakePower();
        motorIntakeLeft.setPower(power);
        motorIntakeRight.setPower(power);

        if (gamepad.transferOpen()) {
            transferPos = TRANSFER_OPEN_POS;
        }
        if (gamepad.transferClose()) {
            transferPos = TRANSFER_CLOSED_POS;
        }
        servoTransfer.setPosition(transferPos);

        telemetry.addData("transfer_pos", transferPos);
    }

    @Override
    public void stop() {
        motorIntakeLeft.setPower(0);
        motorIntakeRight.setPower(0);
    }
}
