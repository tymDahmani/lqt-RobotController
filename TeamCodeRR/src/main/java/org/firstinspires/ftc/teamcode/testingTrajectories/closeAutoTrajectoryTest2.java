package org.firstinspires.ftc.teamcode.testingTrajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/*
trajectory strategy:
        .lineToY(-34.5)
        .waitSeconds(1)
        .setTangent(2 * Math.PI)
        .lineToXLinearHeading(47, 2 * Math.PI)
        .waitSeconds(1)
        .strafeTo(new Vector2d(47, -60))
        .setTangent(2 * Math.PI)
        .lineToX(60)
 */
@Autonomous
public class closeAutoTrajectoryTest2 extends LinearOpMode {

    DcMotor gripperArm;
    Servo gripperL;
    Servo gripperR;
    Servo tilting;

    @Override
    public void runOpMode() throws InterruptedException {

        gripperArm = hardwareMap.get(DcMotor.class, "gripperArm");

        gripperL = hardwareMap.get(Servo.class, "gripperL");
        gripperR = hardwareMap.get(Servo.class, "gripperR");
        tilting = hardwareMap.get(Servo.class, "tilting");

        gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gripperL.setDirection(Servo.Direction.REVERSE);
        gripperArm.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("test test test. im waiting for u to press the start button");
        telemetry.update();

        waitForStart();

        Pose2d beginPose = new Pose2d(12, -64.25, Math.PI / 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToY(-34.5)
                            .waitSeconds(0.5)
                            .build());

        }

        MecanumDrive driveOut = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        if (driveOut.leftFront.isBusy()) {
            idle();
        }

        openArm();

        telemetry.addLine("just making sure we got to this point");
        telemetry.update();

        gripperL.setPosition(0);
        sleep(500);
        gripperL.setPosition(1);

        sleep(2000);

        tilting.setPosition(0.7);

        sleep(500);

        closeArm();

        sleep(1000);

        Pose2d beginPose1 = new Pose2d(12, -34.5, Math.PI / 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose1);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose1)
                            .setTangent(2 * Math.PI)
                            .lineToXLinearHeading(47, 2 * Math.PI)
                            .waitSeconds(0.5)
                            .build());

        }

        if (driveOut.leftFront.isBusy()) {
            idle();
        }


        openArm();

        gripperR.setPosition(0);

        sleep(2000);

        gripperL.setPosition(1);
        gripperR.setPosition(1);

        tilting.setPosition(0.7);

        sleep(500);

        closeArm();

        sleep(1000);

        Pose2d beginPose2 = new Pose2d(47, -34.5, Math.PI * 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose2);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose2)
                            .strafeTo(new Vector2d(47, -60))
                            .setTangent(2 * Math.PI)
                            .lineToX(60)
                            .build());

        }





    }

    void openArm() {
        gripperArm.setTargetPosition(250);
        gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperArm.setPower(0.2);
    }

    void closeArm() {
        gripperArm.setTargetPosition(-250);
        gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperArm.setPower(0.2);
    }

}
