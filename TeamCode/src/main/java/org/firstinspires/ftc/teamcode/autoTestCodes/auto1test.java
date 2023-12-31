package org.firstinspires.ftc.teamcode.autoTestCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class auto1test extends LinearOpMode {
        // robot actuators
    // drive train
    DcMotorEx leftF;
    DcMotorEx leftB;
    DcMotorEx rightF;
    DcMotorEx rightB;

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


        // vars
    // servo position
    int servoPos = 0;


    // pixel pos (left, mid, or right - 1, 2 ,3)
    int tpPos = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeInInit()) {
            // hardware map
            slideL = hardwareMap.dcMotor.get("slide1");
            slideR = hardwareMap.dcMotor.get("slide2");
            armBase = hardwareMap.dcMotor.get("armBase");
            gripperArm = hardwareMap.dcMotor.get("gripper arm");
            tilting = hardwareMap.servo.get("tilting servo");
            gripperL = hardwareMap.servo.get("gripper1");
            gripperR = hardwareMap.servo.get("gripper2");

            // reversed motors
            rightB.setDirection(DcMotorSimple.Direction.REVERSE);
            rightF.setDirection(DcMotorSimple.Direction.REVERSE);
            gripperL.setDirection(Servo.Direction.REVERSE);

            // stop and reset encoder
            leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // run using encoder
            leftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gripperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
        waitForStart();

        while (opModeIsActive()) {
            // turn on the camera. + init the eocv pipeline class or whatever so it would be ready to work - better use a function for this or a whole class

            // move forward to read the tp
            move(0, 0);

            // read the team prop (either w eocv or ml)
            // make sure to store the class's returnings for the tp pos in this var (1=left, 2=mid, 3=right):
            tpPos = 0;

            // turn to the tp pos detected by the camera
            move(0, 0);

            // put the purple pixel depending on the index given by the reads of the camera
            putPurplePixel(servoPos);

            // go to the back drop
            move(0, 0);

            // move the gripper arm and use the gripper to drop the pixel
            backDrop(0, 0, 0);

            // park
            move(0, 0);

        }
    }

    void move(int dis, int vel) {

        leftF.setTargetPosition(dis);
        leftB.setTargetPosition(dis);
        rightF.setTargetPosition(dis);
        rightB.setTargetPosition(dis);

        leftF.setVelocity(vel);
        leftB.setVelocity(vel);
        rightF.setVelocity(vel);
        rightB.setVelocity(vel);

        while (leftF.isBusy()) {
            idle();
        }

        // let the robot calm down
        sleep(100);

    }


    void putPurplePixel (int pos) {

        // open
        gripperR.setPosition(pos);

        // close
        gripperR.setPosition(pos);


        sleep(100);

    }

    void backDrop (int dis, int pwr, int pos) {

        // arm tar pos to be on the back drop
        gripperArm.setTargetPosition(dis);

        // move arm
        gripperArm.setPower(pwr);

        while (gripperArm.isBusy()) {
            idle();
        }

        // yellow pixel servo move
        gripperL.setPosition(pos);

        sleep(100);

    }


}
