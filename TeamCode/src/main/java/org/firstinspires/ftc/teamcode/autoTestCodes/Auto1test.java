package org.firstinspires.ftc.teamcode.autoTestCodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.eocvDetect.blobDetectionTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


// NOTE: the driveEncoder method can be used instead of the move method to move the robot.
// the driveEncoder method takes the inputs: motor speed, left motor meters (for both front and back), right meters, timeout. and calculates the ticks needed and
// applies the to the motors and gives the speed in the parameters and makes the motors move.
// this is an example line:
// encoderDrive(DRIVE_SPEED, 1, 1, 5.0); // S1: Forward 1 meter with 5 Sec timeout

@Autonomous
public class Auto1test extends LinearOpMode {

    OpenCvCamera webCam;

        // robot actuators
    // drive train
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;

    // slides
    DcMotor slideL;
    DcMotor slideR;

    // arm base core
    DcMotor armBase;

    // gripper arm core
    DcMotor gripperArm;

    // gripper tilting servo
    CRServo tilting;

    // grippers
    Servo gripperL;
    Servo gripperR;

    //elapsed time
    ElapsedTime runtime = new ElapsedTime();




        // vars
    // servo position
    int servoPos = 0;


    // pixel pos (left, mid, or right - 1, 2 ,3)
    int tpPos = 2;

    // vars for the driveEncoder method - which calculates the ticks needed for the given distance, time and which motors
    static final double tpr = 1000; // ticks per revolution
    
    static final double MOTOR_GEAR_REDUCTION = 1.0; // sth about the motor gears and ratios (This is < 1.0 if geared UP)
    static final double WHEEL_DIAMETER_CM = 7.0; // For figuring circumference
    static final double cpMeter = (tpr * MOTOR_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415); // cmp = counts per meter
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;






    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        blobDetectionTest colorDetector = new blobDetectionTest(telemetry);
        webCam.setPipeline(colorDetector);


            // hardware map
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            slideL = hardwareMap.get(DcMotor.class, "SlideL");
            slideR = hardwareMap.get(DcMotor.class,"SlideR");
            armBase = hardwareMap.get(DcMotor.class,"armBase");
            gripperArm = hardwareMap.get(DcMotor.class,"gripperArm");
            tilting = hardwareMap.get(CRServo.class,"tilting");
            gripperL = hardwareMap.get(Servo.class,"gripperL");
            gripperR = hardwareMap.get(Servo.class,"gripperR");

            // reversed motors
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            gripperL.setDirection(Servo.Direction.REVERSE);

            // stop and reset encoder
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // run using encoder
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gripperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened() {
                    // Usually this is where you'll want to start streaming from the camera (see section 4) this is in the easy opencv docs
                    webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);


                }
                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                    telemetry.addLine("the camera is dead! help!");
                }
            });

        // make sure to store the class's returnings for the tp pos in this var (1=left, 2=mid, 3=right):
        switch (blobDetectionTest.getTp_zone()) {
            case LEFT:
                // code to be executed if expression == LEFT
                telemetry.addLine("position detected is left");
                tpPos = 1;
                break;
            case MID:
                // code to be executed if expression == MID
                telemetry.addLine("position detected is mid");
                tpPos = 2;
                break;
            // more cases as needed
            case RIGHT:
                // code to be executed if none of the cases match
                telemetry.addLine("position detected is right");
                tpPos = 3;
        }









        waitForStart();

        while (opModeIsActive()) {
            // turn on the camera. + init the eocv pipeline class or whatever so it would be ready to work - better use a function for this or a whole class
            // done above in the init step

            // move forward to read the tp
//            move(0, 0, 0, 0, 0);

            // read the team prop (either w eocv or ml)
                // if the camera read nothing put it in the mid (default pos), which means the default pos num will be 2

            webCam.stopStreaming();


            // turn to the tp pos detected by the camera
            if (tpPos == 1) {
                // turn movements to face the left spike mark
//                leftSpikeMark(1, 0.4);

                move(2000, 0.2, 0.2, 0.2, 0.2);
            } else if (tpPos == 2) {
                // turn movements to face the mid spike mark
//                midSpikeMark(0, 0);

                move(2000, -0.2, -0.2, 0.2, 0.2);
            } else {
                // turn movements to face the right spike mark
//                rightSpikeMark(0, 0);

                move(2000, -0.2, -0.2, -0.2, -0.2);
            }


            // put the purple pixel depending on the index given by the reads of the camera
//            putPurplePixel(servoPos);

            // go to the back drop on the column detected by the camera
//            if (tpPos == 1) {
//                // turn movements to face the left spike mark
//                leftSpikeMark(0, 0);
//
//            } else if (tpPos == 2) {
//                // turn movements to face the mid spike mark
//                midSpikeMark(0, 0);
//
//            } else {
//                // turn movements to face the right spike mark
//                rightSpikeMark(0, 0);
//
//            }

            // move the gripper arm and use the gripper to drop the pixel
//            backDrop(0, 0, 0);

            // park
//            move(0, 0, 0, 0, 0);

        }
    }

    void move(int dis, double vel, double vel1, double vel2, double vel3) {

        leftFront.setTargetPosition(dis);
        leftBack.setTargetPosition(dis);
        rightFront.setTargetPosition(dis);
        rightBack.setTargetPosition(dis);

        leftFront.setVelocity(vel);
        leftBack.setVelocity(vel1);
        rightFront.setVelocity(vel2);
        rightBack.setVelocity(vel3);

        while (leftFront.isBusy()) {
            idle();
        }

        // let the robot calm down
        sleep(100);

    }

    void leftSpikeMark (int dis, double vel) {
        leftFront.setTargetPosition(dis);
        leftBack.setTargetPosition(dis);
        rightFront.setTargetPosition(dis);
        rightBack.setTargetPosition(dis);

        leftFront.setVelocity(vel);
        leftBack.setVelocity(vel);
        rightFront.setVelocity(vel);
        rightBack.setVelocity(vel);

        while (leftFront.isBusy()) {
            idle();
        }

        // let the robot calm down
        sleep(100);

    }

    void midSpikeMark (int dis, double vel) {
        leftFront.setTargetPosition(dis);
        leftBack.setTargetPosition(dis);
        rightFront.setTargetPosition(dis);
        rightBack.setTargetPosition(dis);

        leftFront.setVelocity(vel);
        leftBack.setVelocity(vel);
        rightFront.setVelocity(vel);
        rightBack.setVelocity(vel);

        while (leftFront.isBusy()) {
            idle();
        }

        // let the robot calm down
        sleep(100);

    }

    void rightSpikeMark (int dis, double vel) {
        leftFront.setTargetPosition(dis);
        leftBack.setTargetPosition(dis);
        rightFront.setTargetPosition(dis);
        rightBack.setTargetPosition(dis);

        leftFront.setVelocity(vel);
        leftBack.setVelocity(vel);
        rightFront.setVelocity(vel);
        rightBack.setVelocity(vel);

        while (leftFront.isBusy()) {
            idle();
        }

        // let the robot calm down
        sleep(100);

    }


    void putPurplePixel (int pos) {

        // open
        gripperR.setPosition(pos);

        // close
        gripperR.setPosition(-pos);


        sleep(100);

    }

    void backDrop (int dis, double pwr, int pos) {

        // arm tar pos to be on the back drop
        gripperArm.setTargetPosition(dis);

        // move arm
        gripperArm.setPower(pwr);

        while (gripperArm.isBusy()) {
            idle();
        }

        // yellow pixel servo move
        gripperL.setPosition(pos);

        // close
        gripperL.setPosition(-pos);


        sleep(100);

    }

    public void encoderDrive(double speed, double leftMeters, double rightMeters, double timeoutS) {

        int leftFrontTar;
        int leftBackTar;
        int rightFrontTar;
        int rightBackTar;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftFrontTar = leftFront.getCurrentPosition() + (int) (leftMeters * cpMeter);
            leftBackTar = leftBack.getCurrentPosition() + (int) (leftMeters * cpMeter);
            rightFrontTar = leftFront.getCurrentPosition() + (int) (rightMeters * cpMeter);
            rightBackTar = leftBack.getCurrentPosition() + (int) (rightMeters * cpMeter);
            leftFront.setTargetPosition(leftFrontTar);
            leftBack.setTargetPosition(leftBackTar);
            rightFront.setTargetPosition(rightFrontTar);
            rightBack.setTargetPosition(rightBackTar);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop. This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && leftBack.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Ticks needed to be ticked", "Running to %7d :%7d", leftFrontTar, rightFrontTar);
                telemetry.addData("ticks ticked", "Running at %7d :%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
