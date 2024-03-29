package org.firstinspires.ftc.teamcode.eocvDetect;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.eocvDetect.pipeline_test;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class pipelineDetectTest_opmode extends LinearOpMode {

    OpenCvCamera webCam;

//    WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCam1");


    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline_test detector = new pipeline_test(telemetry);
        webCam.setPipeline(detector);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4) this is in the easy opencv docs
                webCam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);


            }
            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("the camera is dead! help!");
            }
        });

        waitForStart();

        switch (pipeline_test.getPos_of_tp()) {
            case LEFT:
                // code to be executed if expression == LEFT
                telemetry.addLine("position detected is left");
                break;
            case MID:
                // code to be executed if expression == MID
                telemetry.addLine("position detected is mid");
                break;
            // more cases as needed
            case RIGHT:
                // code to be executed if none of the cases match
                telemetry.addLine("position detected is right");
        }

        webCam.stopStreaming();



    }
}
