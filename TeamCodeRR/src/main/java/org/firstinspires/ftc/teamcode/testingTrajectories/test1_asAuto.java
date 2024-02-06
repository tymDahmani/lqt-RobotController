package org.firstinspires.ftc.teamcode.testingTrajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

public final class test1_asAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(12, -70, Math.PI / 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToY(-34.5)
                            .waitSeconds(2)
                            .splineToLinearHeading(new Pose2d(49.87, -35.5, 2 * Math.PI), 0)
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(49.87, -60))
                            .setTangent(2 * Math.PI)
                            .lineToY(-61.37)
                            .build());

        }
        while (opModeIsActive()) {
            telemetry.addData("who's the most perfect and best mentor ever? ", "you :) (\"malek\" so u wont overthink it lol)");


        }





    }
}

