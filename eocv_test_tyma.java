package org.firstinspires.ftc.teamcode.openCV__autoDetect;

import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

import static java.lang.Thread.sleep;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;


@Autonomous
public class eocv_test_tyma extends OpMode {

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
                webCam1.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }


    @Override
    public void loop() {
        /*
         * Send some stats to the telemetry
         */
        telemetry.addData("Frame Count", webCam1.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webCam1.getFps()));
        telemetry.addData("Total frame time ms", webCam1.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webCam1.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webCam1.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webCam1.getCurrentPipelineMaxFps());
        telemetry.update();


    }

    class pipeline extends OpenCvPipeline {

        Mat grey = new Mat();

        boolean toggleRecording = false;



        public void init(Mat input) {
            // Executed before the first call to processFrame
        }


        public Mat processFrame(Mat input) {
            // Executed every time a new frame is dispatched

            // this - as the docs said \-.-/ - will convert the view from the camera into black & white
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);


            // Draws a simple box around the middle 1/2 of the entire frame:
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    // lavender is the color of the rectangle :3
                    new Scalar(230, 230, 250), 4);

            return input; // Return the image that will be displayed in the viewport
            // (In this case the input mat directly)

        }


        public void onViewportTapped() {
            // Executed when the image display is clicked by the mouse or tapped
            // This method is executed from the UI thread, so be careful to not
            // perform any sort heavy processing here! Your app might hang otherwise

            toggleRecording = !toggleRecording;

            if(toggleRecording) {
                /*
                 * This is all you need to do to start recording.
                 */
                webCam1.startRecordingPipeline(
                        new PipelineRecordingParameters.Builder()
                                .setBitrate(4, PipelineRecordingParameters.BitrateUnits.Mbps)
                                .setEncoder(PipelineRecordingParameters.Encoder.H264)
                                .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                                .setFrameRate(30)
                                .setPath(Environment.getExternalStorageDirectory().getPath())
                                .build());
            }
            else
            {
                /*
                 * Note: if you don't stop recording by yourself, it will be automatically
                 * stopped for you at the end of your OpMode
                 */
                webCam1.stopRecordingPipeline();
            }

        }
    }
}


////////////////taymaaaa iz da bezt programmer
//my mentor iz da bezt mentor ewer :3
