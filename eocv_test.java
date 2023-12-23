package org.firstinspires.ftc.teamcode.openCV__autoDetect;

import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class eocv_test extends OpMode {

    OpenCvWebcam webCam1 = null;

    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webCam1.setPipeline(new pipeline());

        webCam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /* input the camera stream details
                // The first two parameters are the desired width and height of the image stream, which are one of these:
                // 320x240
                // 640x480
                // 1280x720
                // 1920x1080
                The third parameter specifies the orientation the camera is being used in. So for instance,
                if you have a phone mounted in portrait on your robot, or a webcam used in its normal orientation, use UPRIGHT.
                But if you have the phone mounted in landscape on your robot, use SIDEWAYS_LEFT or SIDEWAYS_RIGHT depending on the specific way you've mounted it.
                SIDEWAYS_LEFT refers to the orientation where the screen is facing toward you in landscape mode and the USB port is facing to the right.

                 example: camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                 */
                webCam1.startStreaming(0, 0, UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop() {

    }

    class pipeline extends OpenCvPipeline {
        public Mat processFrame(Mat input) {
            return input;

        }
    }
}
