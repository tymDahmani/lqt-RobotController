package org.firstinspires.ftc.teamcode.testingTrajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public final class TrajectoryAndArm extends LinearOpMode {


    // slides
    DcMotor slideL;
    DcMotor slideR;

    // arm base core
    DcMotor armBase;

    // gripper arm core
    DcMotor gripperArm;

    // gripper tilting servo
    Servo tilting;

    // grippers
    Servo gripperL;
    Servo gripperR;

    @Override
    public void runOpMode() throws InterruptedException {

        slideL = hardwareMap.get(DcMotor.class, "SlideL");
        slideR = hardwareMap.get(DcMotor.class, "SlideR");
        armBase = hardwareMap.get(DcMotor.class, "armBase");
        gripperArm = hardwareMap.get(DcMotor.class, "gripperArm");
        tilting = hardwareMap.get(Servo.class, "tilting");
        gripperL = hardwareMap.get(Servo.class, "gripperL");
        gripperR = hardwareMap.get(Servo.class, "gripperR");

        ((DcMotorEx) armBase).setVelocityPIDFCoefficients(40, 0.1, 0.1, 50);
        ((DcMotorEx) armBase).setPositionPIDFCoefficients(5);
        ((DcMotorEx) gripperArm).setVelocityPIDFCoefficients(40, 0.1, 0.1, 50);
        ((DcMotorEx) gripperArm).setPositionPIDFCoefficients(5);
        // reversed motors
        gripperL.setDirection(Servo.Direction.REVERSE);
        gripperArm.setDirection(DcMotorSimple.Direction.REVERSE);


        // stop and reset encoder
        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        Pose2d beginPose = new Pose2d(-34, -70, Math.PI / 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);




            Actions.runBlocking(
                    drive.actionBuilder(beginPose)

                            .lineToY(-45)
                            .waitSeconds(1)
                            .build());

            ArmBase(200, 0.5);
//            sleep(2000);
            Open_Tilting();
            sleep(500);
            GripperArm(300, 0.2);
            sleep(2000);
            gripperR.setPosition(0);
            sleep(500);

//            Close_Tilting();
            GripperArm(80, 0.2);
            sleep(500);
            gripperR.setPosition(1);


            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-34, -45, Math.PI / 2))

                            .splineToLinearHeading(new Pose2d(-50, -35.5, 1 * Math.PI), 0)
//                            .waitSeconds(2)
                            .build());


            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-50, -35.5, 1 * Math.PI))
                            .strafeTo(new Vector2d(-50, -10))
                            .strafeTo(new Vector2d(20, -10))
                            .splineToLinearHeading(new Pose2d(40, -35.5, 2 * Math.PI), 1)
                            .build());

            ArmBase(400, 0.5);
//            sleep(1000);
            GripperArm(220, 0.2);
            sleep(2000);

            Open_Tilting();
            sleep(1000);

            gripperL.setPosition(0);
            sleep(500);
            gripperL.setPosition(1);

            GripperArm(10, 0.2);
            sleep(500);

            ArmBase(10, 0.5);
            sleep(2000);



        }
    }

    private void ArmBase(int Ticks, double Power) {
        armBase.setTargetPosition(Ticks);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setPower(Power);
    }


    private void GripperArm(int Ticks2, double Power2) {
        gripperArm.setTargetPosition(Ticks2);
        gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperArm.setPower(Power2);
    }

    private void Open_GripperL() {
        gripperR.setPosition(0);
        gripperL.setPosition(0);
        sleep(200);
    }


    private void Close_Tilting() {
        tilting.setPosition(0.4);
        sleep(200);
    }

    private void Open_Tilting() {
        tilting.setPosition(0.7);
        sleep(200);
    }


    private void Close_GripperL() {
        gripperR.setPosition(1);
        gripperL.setPosition(1);
        sleep(200);

    }
}


