package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class LeftFinal extends LinearOpMode {

    private DcMotor f_l;
    private DcMotor f_r;
    private DcMotor r_l;
    private DcMotor r_r;
    private Servo right_ntk;
    private Servo left_ntk;
    private DcMotor r_motion_motor;

    OpenCvCamera camera;
    LeftFinal aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int Left = 17;
    int Middle = 18;
    int Right = 19;

    AprilTagDetection tagOfInterest = null;


    int Drive_Seed;
    int Drive_Speed_2;
    @Override
    public void runOpMode() {

        /////////////
        f_l = hardwareMap.get(DcMotor.class, "f_l");
        f_r = hardwareMap.get(DcMotor.class, "f_r");
        r_l = hardwareMap.get(DcMotor.class, "r_l");
        r_r = hardwareMap.get(DcMotor.class, "r_r");
        right_ntk = hardwareMap.get(Servo.class, "right_ntk");
        left_ntk = hardwareMap.get(Servo.class, "left_ntk");
        r_motion_motor = hardwareMap.get(DcMotor.class, "r_motion_motor");

        f_l.setDirection(DcMotorSimple.Direction.REVERSE);
        r_l.setDirection(DcMotorSimple.Direction.REVERSE);

        right_ntk.setDirection(Servo.Direction.REVERSE);
        f_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        f_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        f_l.setDirection(DcMotorSimple.Direction.REVERSE);
        r_l.setDirection(DcMotorSimple.Direction.REVERSE);
        right_ntk.setPosition(0.45);
        left_ntk.setPosition(0.55);
        Drive_Seed = 1900;
        Drive_Speed_2 = 1700;
        //////////////////////6437859201745629017465829013746589201837456920183745639201837465290837465829##########@$%^&$#@$%^&


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
//////////////////////
        if (tagOfInterest == null || tagOfInterest.id == Left) {
            ////// Left Code Here

            DriveTrain(2380, Drive_Speed_2, 2380, Drive_Speed_2, 2380, Drive_Speed_2, 2380, Drive_Speed_2);
            Slides(2850, 2300);
            sleep(250);
            DriveTrain(-1120, Drive_Speed_2, -1120, Drive_Speed_2, -1120, Drive_Speed_2, -1120, Drive_Speed_2);
            sleep(250);
            DriveTrain(537, Drive_Speed_2, 537, Drive_Speed_2, -537, Drive_Speed_2, -537, Drive_Speed_2);
            sleep(250);
            DriveTrain(450, Drive_Speed_2, 450, Drive_Speed_2, 450, Drive_Speed_2, 450, Drive_Speed_2);
            sleep(500);
            Open_Gripper();
            sleep(500);
            DriveTrain(-450, Drive_Speed_2, -450, Drive_Speed_2, -450, Drive_Speed_2, -450, Drive_Speed_2);
            //Slides(630, 2000);
            //sleep(300);
            DriveTrain(-537, Drive_Speed_2, -537, Drive_Speed_2, 537, Drive_Speed_2, 537, Drive_Speed_2);
            //sleep(250);
            //DriveTrain(1030, Drive_Speed_2, 1030, Drive_Speed_2, 1030, Drive_Speed_2, 1030, Drive_Speed_2);
            sleep(250);
            DriveTrain(-1085, Drive_Speed_2, -1085, Drive_Speed_2, 1085, Drive_Speed_2, 1085, Drive_Speed_2);
            //sleep(250);
            //Loop1();
            //Slides(520, 2000);
            //Loop1();
            //Slides(390, 2000);
            //Loop1();
            Slides(0, 3000);

            // newcode here
            // Move FW to catch the Cone
            DriveTrain(1000, 3000, 1000, 3000, 1000, 3000, 1000, 3000);

        }
        //       else if (tagOfInterest == null || tagOfInterest.id == Middle)
        else if (tagOfInterest.id == Middle) {

            // Middle Code Here
            DriveTrain(2380, Drive_Speed_2, 2380, Drive_Speed_2, 2380, Drive_Speed_2, 2380, Drive_Speed_2);
            Slides(2850, 2300);
            sleep(250);
            DriveTrain(-1120, Drive_Speed_2, -1120, Drive_Speed_2, -1120, Drive_Speed_2, -1120, Drive_Speed_2);
            sleep(250);
            DriveTrain(537, Drive_Speed_2, 537, Drive_Speed_2, -537, Drive_Speed_2, -537, Drive_Speed_2);
            sleep(250);
            DriveTrain(450, Drive_Speed_2, 450, Drive_Speed_2, 450, Drive_Speed_2, 450, Drive_Speed_2);
            sleep(500);
            Open_Gripper();
            sleep(500);
            DriveTrain(-450, Drive_Speed_2, -450, Drive_Speed_2, -450, Drive_Speed_2, -450, Drive_Speed_2);
            //Slides(630, 2000);
            sleep(300);
            DriveTrain(-537, Drive_Speed_2, -537, Drive_Speed_2, 537, Drive_Speed_2, 537, Drive_Speed_2);
            sleep(250);
            //DriveTrain(1030, Drive_Speed_2, 1030, Drive_Speed_2, 1030, Drive_Speed_2, 1030, Drive_Speed_2);
            sleep(250);
            //DriveTrain(-1085, Drive_Speed_2, -1085, Drive_Speed_2, 1085, Drive_Speed_2, 1085, Drive_Speed_2);
            sleep(250);
            //Loop1();
            //Slides(520, 2000);
            //Loop1();
            //Slides(390, 2000);
            //Loop1();
            Slides(0, 3000);
            // Move FW to catch the Cone



        } else if (tagOfInterest.id == Right) {
            // Right Code Here

            DriveTrain(2380, Drive_Speed_2, 2380, Drive_Speed_2, 2380, Drive_Speed_2, 2380, Drive_Speed_2);
            Slides(2850, 2300);
            sleep(250);
            DriveTrain(-1120, Drive_Speed_2, -1120, Drive_Speed_2, -1120, Drive_Speed_2, -1120, Drive_Speed_2);
            sleep(250);
            DriveTrain(537, Drive_Speed_2, 537, Drive_Speed_2, -537, Drive_Speed_2, -537, Drive_Speed_2);
            sleep(250);
            DriveTrain(450, Drive_Speed_2, 450, Drive_Speed_2, 450, Drive_Speed_2, 450, Drive_Speed_2);
            sleep(500);
            Open_Gripper();
            sleep(500);
            DriveTrain(-450, Drive_Speed_2, -450, Drive_Speed_2, -450, Drive_Speed_2, -450, Drive_Speed_2);
            //Slides(630, 2000);
            sleep(300);
            DriveTrain(-537, Drive_Speed_2, -537, Drive_Speed_2, 537, Drive_Speed_2, 537, Drive_Speed_2);
            sleep(250);
            //DriveTrain(1030, Drive_Speed_2, 1030, Drive_Speed_2, 1030, Drive_Speed_2, 1030, Drive_Speed_2);
            sleep(250);
            DriveTrain(-1085, Drive_Speed_2, -1085, Drive_Speed_2, 1085, Drive_Speed_2, 1085, Drive_Speed_2);
            sleep(250);
            //Loop1();
            //Slides(520, 2000);
            //Loop1();
            //Slides(390, 2000);
            //Loop1();
            Slides(0, 3500);
            // Move FW to catch the Cone
            DriveTrain(-1000, 3000, -1000, 1000, -1000, 3000, -1000, 3000);
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void DriveTrain(int Ticks_1, int Power_1, int Ticks_2, int Power_2, int Ticks_3, int Power_3, int Ticks_4, int Power_4) {
        f_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        f_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_l.setTargetPosition(Ticks_1);
        f_l.setTargetPosition(Ticks_2);
        f_r.setTargetPosition(Ticks_3);
        r_r.setTargetPosition(Ticks_4);
        r_l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        f_l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        f_r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r_r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) r_l).setVelocity(Power_1);
        ((DcMotorEx) f_l).setVelocity(Power_2);
        ((DcMotorEx) f_r).setVelocity(Power_3);
        ((DcMotorEx) r_r).setVelocity(Power_4);
        while (f_l.isBusy() && f_r.isBusy() && r_l.isBusy() && r_r.isBusy()) {
            idle();
        }
    }

    private void Slides(int Ticks, int Speed) {
        r_motion_motor.setTargetPosition(Ticks);
        r_motion_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) r_motion_motor).setVelocity(Speed);
    }

    private void Close_Gripper() {
        right_ntk.setPosition(0.45);
        left_ntk.setPosition(0.55);
        sleep(200);
    }

    /**
     * Describe this function...
     */
    private void Open_Gripper() {
        right_ntk.setPosition(1);
        left_ntk.setPosition(0);
        sleep(200);
    }

    private void Loop1() {
        // Move FW to catch the Cone
        DriveTrain(1275, Drive_Seed, 1275, Drive_Seed, 1275, Drive_Seed, 1275, Drive_Seed);
        Close_Gripper();
        sleep(400);
        Slides(1800, 3000);
        sleep(250);
        DriveTrain(-1250, Drive_Seed, -1250, Drive_Seed, -1250, Drive_Seed, -1250, Drive_Seed);
        sleep(250);
        DriveTrain(600, Drive_Seed, -600, Drive_Seed, 600, Drive_Seed, -600, Drive_Seed);
        sleep(250);
        DriveTrain(252, Drive_Seed, 252, Drive_Seed, 252, Drive_Seed, 252, Drive_Seed);
        sleep(100);
        Open_Gripper();
        sleep(250);
        DriveTrain(-252, Drive_Seed, -252, Drive_Seed, -252, Drive_Seed, -252, Drive_Seed);
        sleep(250);
        DriveTrain(-600, Drive_Seed, 600, Drive_Seed, -600, Drive_Seed, 600, Drive_Seed);
        sleep(250);
    }
    private void Loop2() {
        // Move FW to catch the Cone
        DriveTrain(1233, Drive_Seed, 1233, Drive_Seed, 1233, Drive_Seed, 1233, Drive_Seed);
        Close_Gripper();
        sleep(400);
        Slides(1800, 3000);
        sleep(250);
        DriveTrain(-1233, Drive_Seed, -1233, Drive_Seed, -1233, Drive_Seed, -1233, Drive_Seed);
        sleep(250);
        DriveTrain(650, Drive_Seed, -650, Drive_Seed, 650, Drive_Seed, -650, Drive_Seed);
        sleep(250);
        DriveTrain(252, Drive_Seed, 252, Drive_Seed, 252, Drive_Seed, 252, Drive_Seed);
        sleep(100);
        Open_Gripper();
        sleep(250);
        DriveTrain(-252, Drive_Seed, -252, Drive_Seed, -252, Drive_Seed, -252, Drive_Seed);
        sleep(250);
        DriveTrain(-650, Drive_Seed, 650, Drive_Seed, -650, Drive_Seed, 650, Drive_Seed);
        sleep(250);
    }
    private void Loop3() {
        // Move FW to catch the Cone
        DriveTrain(1233, Drive_Seed, 1233, Drive_Seed, 1233, Drive_Seed, 1233, Drive_Seed);
        Close_Gripper();
        sleep(400);
        Slides(1800, 3000);
        sleep(250);
        DriveTrain(-1233, Drive_Seed, -1233, Drive_Seed, -1233, Drive_Seed, -1233, Drive_Seed);
        sleep(250);
        DriveTrain(-650, Drive_Seed, 650, Drive_Seed, -650, Drive_Seed, 650, Drive_Seed);
        sleep(250);
        DriveTrain(252, Drive_Seed, 252, Drive_Seed, 252, Drive_Seed, 252, Drive_Seed);
        sleep(100);
        Open_Gripper();
        sleep(250);
        DriveTrain(-252, Drive_Seed, -252, Drive_Seed, -252, Drive_Seed, -252, Drive_Seed);
        sleep(250);
        DriveTrain(650, Drive_Seed, -650, Drive_Seed, 650, Drive_Seed, -650, Drive_Seed);
        sleep(250);
    }



}