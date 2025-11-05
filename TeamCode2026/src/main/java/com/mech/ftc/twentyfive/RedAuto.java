package com.mech.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

@Autonomous(name = "RedAuto")
public class RedAuto extends OpMode {
    Pose2d beginPose = new Pose2d(0, 0, 0);
    MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


    @Override
    public void init() {

    }

    @Override
    public void start() {
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .build());
    }

    @Override
    public void loop() {


    }
}
