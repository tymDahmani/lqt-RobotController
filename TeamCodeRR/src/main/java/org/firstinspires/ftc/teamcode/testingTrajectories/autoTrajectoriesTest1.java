package org.firstinspires.ftc.teamcode.testingTrajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/* full trajectory for the drivetrain:

.lineToY(-34.5)
.waitSeconds(2)
.splineToLinearHeading(new Pose2d(49.87, -35.5, 2 * Math.PI), 0)
.waitSeconds(2)
.strafeTo(new Vector2d(49.87, -60))

 */
@Autonomous
public class autoTrajectoriesTest1 extends LinearOpMode {

    Servo gripperL;
    DcMotor gripperArm;



    @Override
    public void runOpMode() throws InterruptedException {

        gripperL = hardwareMap.get(Servo.class, "gripperL");
        gripperArm = hardwareMap.get(DcMotor.class, "gripperArm");

        waitForStart();

        Pose2d beginPose = new Pose2d(12, -70, Math.PI / 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToY(-34.5)
                            .waitSeconds(1)
                            .build());



        }

        telemetry.addLine("now should turn to put the purple pixel");
        telemetry.update();

        gripperL.setPosition(1);
        sleep(500);
        gripperL.setPosition(0);

        telemetry.addLine("servo is done moving");
        telemetry.update();

        Pose2d beginPose1 = new Pose2d(12, -34.5, Math.PI / 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose1);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose1)
                            .splineToLinearHeading(new Pose2d(49.87, -35.5, 2 * Math.PI), 0)
                            .waitSeconds(1)
                            .build());




        }

        gripperArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gripperArm.setPower(0.2);
        sleep(500);
        gripperArm.setPower(0);


        Pose2d beginPose2 = new Pose2d(49.87, -35.5, 2 * Math.PI);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose2);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose2)
                            .strafeTo(new Vector2d(49.87, -60))
                            .waitSeconds(1)
                            .build());



        }

        if (isStopRequested()) {
            telemetry.addLine("we're done");
            telemetry.update();
        }


    }
}
