package org.firstinspires.ftc.teamcode.uselessAutoCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.eocvDetect.blobDetectionTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class auto2test extends LinearOpMode {

    // camera
    OpenCvCamera webCam;

        // actuators
    // drivetrain
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

        // vars
    // servo position
    int servoPos = 0;


    // pixel pos (left, mid, or right - 1, 2 ,3)
    int tpPos = blobDetectionTest.tp_zone;

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
        if (tpPos == 1) {
            // code to be executed if position detected = 1 = left
            telemetry.addData("position detected, ", tpPos);
        } else if (tpPos == 2) {
            // code to be executed if position detected = 2 = right
            telemetry.addData("position detected, ", tpPos);
        } else {
            // code to be executed if none of the cases match = right
            telemetry.addData("position detected, ", tpPos);
        }

        telemetry.addData("max(minimum) color distance value", blobDetectionTest.getMaxDistance());
        telemetry.addData("zone where the team prop were found on: ", tpPos);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


        }



    }

    public int ticksCalc(int distance) {

        int Motor_tpr = 560; // with gears (1:20)
        double circumference = 23.5619449;
        double ticksPerCM = Motor_tpr / circumference;
        int TickToDest = (int) (distance * ticksPerCM);
        return TickToDest;

    }


    public void drivetrain(int ticks, double velocity) {
        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFront.setVelocity(velocity);
        leftBack.setVelocity(velocity);
        rightFront.setVelocity(velocity);
        rightBack.setVelocity(velocity);


    }
}
