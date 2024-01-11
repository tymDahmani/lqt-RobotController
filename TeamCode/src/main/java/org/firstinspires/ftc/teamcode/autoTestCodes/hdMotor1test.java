package org.firstinspires.ftc.teamcode.autoTestCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.eocvDetect.blobDetectionTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class hdMotor1test extends LinearOpMode {

    DcMotorEx leftFront;
    
    OpenCvCamera webCam;
    
    int tp_dudu = blobDetectionTest.tp_zone;


    @Override
    public void runOpMode() throws InterruptedException {
        
        leftFront = hardwareMap.get(DcMotorEx.class, "leftF");
        
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
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
            }
        });
        
        if (tp_dudu == 1) {
            telemetry.addData("pos deteceted: ", tp_dudu+" = left zone");
            
        } else if (tp_dudu == 2) {
            telemetry.addData("pos detected: ", tp_dudu+" = mid zone");
            
        } else {
            telemetry.addData("pos detected: ", tp_dudu+" = right zone");
            
        }

        waitForStart();
        
        
        while (opModeIsActive()) {
            leftFront.setTargetPosition(2000);
            
            if (tp_dudu == 1) {
                leftFront.setPower(0.2);
                telemetry.addLine("we're moving forward");

            } else if (tp_dudu == 2) {
                leftFront.setPower(-0.2);
                telemetry.addLine("we're moving rückwärts");

            } else {
                leftFront.setPower(0.1);
                sleep(100);
                leftFront.setPower(-0.1);
                telemetry.addLine("Achtung! we will move forwards and backwords");

            }
            
        }

        webCam.stopStreaming();



    }
    
}
