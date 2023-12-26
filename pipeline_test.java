package org.firstinspires.ftc.teamcode.openCV__autoDetect;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Thread.sleep;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


//@Autonomous
//public class eocv_test_red extends OpMode {
//
//    OpenCvWebcam webCam1 = null;
//
//    @Override
//    public void init() {
//
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCam1");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        webCam1.setPipeline(new pipeline());
//
//        webCam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                /* input the camera stream details
//                // The first two parameters are the desired width and height of the image stream, which are one of these:
//                // 320x240
//                // 640x480
//                // 1280x720
//                // 1920x1080
//                The third parameter specifies the orientation the camera is being used in. So for instance,
//                if you have a phone mounted in portrait on your robot, or a webcam used in its normal orientation, use UPRIGHT.
//                But if you have the phone mounted in landscape on your robot, use SIDEWAYS_LEFT or SIDEWAYS_RIGHT depending on the specific way you've mounted it.
//                SIDEWAYS_LEFT refers to the orientation where the screen is facing toward you in landscape mode and the USB port is facing to the right.
//
//                 example: camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                 */
//                webCam1.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//    }


    public class pipeline_test extends OpenCvPipeline {

        // our camera stream image:
        // webCam1.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);

        Mat red = new Mat();

//        boolean toggleRecording = false;

        // here add the rectangles for each side of the spike marks so the image will be a little more limited for the camera to read
        // please do input the size of the rectangles here instead of the 0's, as they will cover the size of the team prop
        final Rect leftSpike = new Rect(0, 0, 0, 0);
        final Rect midSpike = new Rect(0, 0, 0, 0);
        final Rect rightSpike = new Rect(0, 0, 0, 0);

        // the lowest threshold value needed for the team prop
        double lowThresholdVal = 0.4; // = 40%. below, we'll use an if statement stating that if the value is above 0.4/40%means it is a team prop

        // an index to store the position of the team prop (1; left, 2; mid, 3; right)
//        public int pos_of_tp = 0;

        public enum pos_of_tp {
            LEFT,
            MID,
            RIGHT
        }
        static pos_of_tp pos_of_tp;


        @Override
        public Mat processFrame(Mat input) {
            // Executed every time a new frame is dispatched

//            // this - as the docs said \-.-/ - will convert the view from the camera into black & white
//            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);

            // i wanna convert the rgb values colors range into hsv
            Imgproc.cvtColor(input, red, Imgproc.COLOR_RGB2HSV);

            // 1; hue, 2; saturation, 3; value (check HSV image to know more)
            // the values here must be for the red team prop
            Scalar lowHSV = new Scalar(0, 0, 0);
            Scalar highHSV = new Scalar(0, 0, 0);

            // thresholding - will convert the colors in the range we give to the robot into white and the rest to black
            Core.inRange(red, lowHSV, highHSV, red);

            // extract the rectangles into the image
            Mat left = red.submat(leftSpike);
            Mat mid = red.submat(midSpike);
            Mat right = red.submat(rightSpike);

            // idk sth about calculating the white area of the object or sth, just gotta write it for now (to check what percentage of the matrix became white)
            double leftValue = Core.sumElems(left).val[0] / leftSpike.area() / 255 /* this 255 is the highest scalar of the hsv values */;
            double midValue = Core.sumElems(mid).val[0] / midSpike.area() / 255;
            double rightValue = Core.sumElems(right).val[0] / rightSpike.area() / 255;

            // release, yaay
            left.release();
            mid.release();
            right.release();

//            // telemetry stuff :O to show the values we've got
//            telemetry.addData("left raw value", (int) Core.sumElems(left).val[0]);
//            telemetry.addData("mid raw value", (int) Core.sumElems(mid).val[0]);
//            telemetry.addData("right raw value", (int) Core.sumElems(right).val[0]);
//
//            telemetry.addData("left percentage", Math.round(leftValue * 100) + "%");
//            telemetry.addData("mid percentage", Math.round(midValue * 100) + "%");
//            telemetry.addData("right percentage", Math.round(rightValue * 100) + "%");


            // if the team prop's value's thresholder is higher than the lowest val we defined above, then it is a team prop
            boolean leftTP /* team prop*/ = leftValue > lowThresholdVal;
            boolean midTP = midValue > lowThresholdVal;
            boolean rightTP = rightValue > lowThresholdVal;

            if (leftTP) {
                pos_of_tp = pos_of_tp.LEFT;

            } else if (midTP) {
                pos_of_tp = pos_of_tp.MID;

            } else if (rightTP) {
                pos_of_tp = pos_of_tp.RIGHT;

            } else {
                telemetry.addLine("non detected!");
            }

            telemetry.addData("the detected position for the team prop:", pos_of_tp);
            telemetry.update();


            return input; // Return the image that will be displayed in the viewport
            // (In this case the input mat directly)

        }

        public pipeline_test.pos_of_tp getPos_of_tp() {
            return pos_of_tp;
        }

        //        public void onViewportTapped() {
//            // Executed when the image display is clicked by the mouse or tapped
//            // This method is executed from the UI thread, so be careful to not
//            // perform any sort heavy processing here! Your app might hang otherwise
//
//            toggleRecording = !toggleRecording;
//
//            if(toggleRecording) {
//                /*
//                 * This is all you need to do to start recording.
//                 */
//                webCam1.startRecordingPipeline(
//                        new PipelineRecordingParameters.Builder()
//                                .setBitrate(4, PipelineRecordingParameters.BitrateUnits.Mbps)
//                                .setEncoder(PipelineRecordingParameters.Encoder.H264)
//                                .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
//                                .setFrameRate(30)
//                                .setPath(Environment.getExternalStorageDirectory().getPath())
//                                .build());
//            }
//            else
//            {
//                /*
//                 * Note: if you don't stop recording by yourself, it will be automatically
//                 * stopped for you at the end of your OpMode
//                 */
//                webCam1.stopRecordingPipeline();
//            }
//
//        }
    }



////////////////taymaaaa iz da bezt programmer
//my mentor iz da bezt mentor ewer :3
