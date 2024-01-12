package org.firstinspires.ftc.teamcode.autoTestCodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.eocvDetect.blobDetectionTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class hdMotor1test extends LinearOpMode {

    DcMotorEx SlideL;

    OpenCvCamera webCam;

    int tp_dudu = blobDetectionTest.tp_zone;


    @Override
    public void runOpMode() throws InterruptedException {

        SlideL = hardwareMap.get(DcMotorEx.class, "SlideL");

        SlideL.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // camera init
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        blobDetectionTest pipeline = new blobDetectionTest(telemetry);
        webCam.setPipeline(pipeline);

        // open camera streaming
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
                telemetry.update();

            }
        });

//        if (tp_dudu == 1) {
        //          telemetry.addData("pos deteceted: ", tp_dudu+" = left zone");
        //        telemetry.update();
//
        //      } else if (tp_dudu == 2) {
        //        telemetry.addData("pos detected: ", tp_dudu+" = mid zone");
        //      telemetry.update();

        //} else {
        //  telemetry.addData("pos detected: ", tp_dudu+" = right zone");
        //telemetry.update();

        //       }

        waitForStart();


        while (opModeIsActive()) {


            if (tp_dudu == 1) {
                SlideL.setTargetPosition(500);
                SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideL.setPower(0.2);
                telemetry.addLine("Left Zone we're moving 500 ticks");
                telemetry.update();

            } else if (tp_dudu == 2) {
                SlideL.setTargetPosition(2000);
                SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideL.setPower(0.2);
                telemetry.addLine("Mid Zone we're moving 2000 ticks ");
                telemetry.update();

            } else {
                SlideL.setTargetPosition(5000);
                SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideL.setPower(0.2);
                telemetry.addLine("Right Zone Achtung! we will move 5000 ticks");
                telemetry.update();

            }

        }

        webCam.stopStreaming();



    }

}