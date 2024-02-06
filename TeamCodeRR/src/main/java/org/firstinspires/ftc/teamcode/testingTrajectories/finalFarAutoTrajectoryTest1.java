package org.firstinspires.ftc.teamcode.testingTrajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.classes.blobDetectionTest;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
///// LEFT
                .lineToYLinearHeading(-34.5, 5 * Math.PI / 6)
                .waitSeconds(4) // drop pixel

                .turn(Math.PI / 6)
                .strafeTo(new Vector2d(-35.5, -11.5))
                .setTangent(Math.PI)
                .lineToX(-52.75)
                .waitSeconds(3) // pick a white pixel

                .setTangent(Math.PI / 2)
                .lineToYLinearHeading(-10.5, 2 * Math.PI)
                .setTangent(2 * Math.PI)
                .lineToXLinearHeading(47, 2 * Math.PI)
                .strafeTo(new Vector2d(47, -34.5))
                .waitSeconds(5) // drop yellow pixel in bd

                .strafeTo(new Vector2d(47, -11.5))
                .setTangent(2 * Math.PI)
                .lineToX(60) // parking

///// MID
                .lineToY(-34.5)
                .waitSeconds(4) // drop pixel
                .setTangent(Math.PI)
                .lineToXLinearHeading(-52.75, Math.PI)
                .waitSeconds(3) // pick a white pixel from stack
                .setTangent(Math.PI / 2)
                .lineToYLinearHeading(-10.5, 2 * Math.PI)
                .waitSeconds(0.3)
                .setTangent(2 * Math.PI)
                .lineToXLinearHeading(47, 2 * Math.PI)
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(47, -34.5))
                .waitSeconds(5) // drop yellow pixel in bd
                .strafeTo(new Vector2d(47, -11.5))
                .setTangent(2 * Math.PI)
                .lineToX(60) // parking

///// RIGHT
                .lineToYLinearHeading(-34.5, Math.PI / 6)
                .waitSeconds(4) // drop pixel
                .setTangent(Math.PI)
                .lineToXLinearHeading(-52.75, Math.PI)
                .waitSeconds(3) // pick a white pixel
                .setTangent(Math.PI / 2)
                .lineToYLinearHeading(-10.5, 2 * Math.PI)
                .setTangent(2 * Math.PI)
                .lineToXLinearHeading(47, 2 * Math.PI)
                .strafeTo(new Vector2d(47, -34.5))
                .waitSeconds(5) // drop yellow pixel in bd
                .strafeTo(new Vector2d(47, -11.5))
                .setTangent(2 * Math.PI)
                .lineToX(60) // parking

 */

public class finalFarAutoTrajectoryTest1 extends LinearOpMode {

    blobDetectionTest detector = new blobDetectionTest(telemetry);
    OpenCvCamera webCam;

    int tpPos = blobDetectionTest.tp_zone;

    @Override
    public void runOpMode() throws InterruptedException {

        // webCam and pipeline init
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

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

        telemetry.addLine("tpPos's indexes; 1=left, 2=mid, 3=right ");
        telemetry.addLine("");
        telemetry.addData("team prop's zone detected was ", tpPos);
        telemetry.update();

        if (tpPos == 1) {
            Pose2d beginPose = new Pose2d(12, -64.25, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .lineToYLinearHeading(-34.5, 5 * Math.PI / 6)
                                .build());
            }

            // drop purple pixel here

            sleep(2000);

            Pose2d beginPose2 = new Pose2d(12, -34.5, 5 * Math.PI / 6);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose2);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose2)
                                .turn(Math.PI / 6)
                                .strafeTo(new Vector2d(-35.5, -11.5))
                                .setTangent(Math.PI)
                                .lineToX(-52.75)
                                .build());
            }

            sleep(2000);

            // pick a white pixel

            Pose2d beginPose3 = new Pose2d(-52.75, -34.5, Math.PI);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose3);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose3)
                                .setTangent(Math.PI / 2)
                                .lineToYLinearHeading(-10.5, 2 * Math.PI)
                                .setTangent(2 * Math.PI)
                                .lineToXLinearHeading(47, 2 * Math.PI)
                                .strafeTo(new Vector2d(47, -34.5))
                                .build());
            }

            // drop yellow pixel

            sleep(2000);

            Pose2d beginPose4 = new Pose2d(-52.75, -34.5, Math.PI);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose4);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose4)
                                .strafeTo(new Vector2d(47, -11.5))
                                .setTangent(2 * Math.PI)
                                .lineToX(60) // parking
                                .build());
            }

            // the robot already parked




        } else if (tpPos == 2) {
            Pose2d beginPose = new Pose2d(12, -64.25, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .lineToY(-34.5)
                                .build());
            }

            // drop purple pixel here

            sleep(2000);

            Pose2d beginPose2 = new Pose2d(12, -34.5, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose2);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose2)
                                .setTangent(Math.PI)
                                .lineToXLinearHeading(-52.75, Math.PI)
                                .build());
            }

            sleep(2000);

            // pick a white pixel

            Pose2d beginPose3 = new Pose2d(-52.75, -34.5, Math.PI);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose3);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose3)
                                .setTangent(Math.PI / 2)
                                .lineToYLinearHeading(-10.5, 2 * Math.PI)
                                .setTangent(2 * Math.PI)
                                .lineToXLinearHeading(47, 2 * Math.PI)
                                .strafeTo(new Vector2d(47, -34.5))
                                .build());
            }

            // drop yellow pixel

            sleep(2000);

            Pose2d beginPose4 = new Pose2d(47, -34.5, Math.PI);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose4);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose4)
                                .strafeTo(new Vector2d(47, -11.5))
                                .setTangent(2 * Math.PI)
                                .lineToX(60) // parking
                                .build());
            }

            // the robot already parked
        } else {
            Pose2d beginPose = new Pose2d(12, -64.25, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .lineToYLinearHeading(-34.5, Math.PI / 6)
                                .build());
            }

            // drop purple pixel here

            sleep(2000);

            Pose2d beginPose2 = new Pose2d(12, -34.5, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose2);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose2)
                                .setTangent(Math.PI)
                                .lineToXLinearHeading(-52.75, Math.PI)
                                .build());
            }

            sleep(2000);

            // pick a white pixel

            Pose2d beginPose3 = new Pose2d(-52.75, -34.5, Math.PI);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose3);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose3)
                                .setTangent(Math.PI / 2)
                                .lineToYLinearHeading(-10.5, 2 * Math.PI)
                                .setTangent(2 * Math.PI)
                                .lineToXLinearHeading(47, 2 * Math.PI)
                                .strafeTo(new Vector2d(47, -34.5))
                                .build());
            }

            // drop yellow pixel

            sleep(2000);

            Pose2d beginPose4 = new Pose2d(47, -34.5, Math.PI);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose4);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose4)
                                .strafeTo(new Vector2d(47, -11.5))
                                .setTangent(2 * Math.PI)
                                .lineToX(60) // parking
                                .build());
            }
        }

        webCam.stopStreaming();





    }
}
