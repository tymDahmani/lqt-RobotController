package org.firstinspires.ftc.teamcode.testingTrajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public final class forward_turn_forwardTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(0, 0, Math.PI / 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
//                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                            .splineTo(new Vector2d(0, 60), Math.PI)
//                            .lineToX(48)
//                            .waitSeconds(3)
                            .lineToY(-24)
                            .waitSeconds(3)
                            .splineTo(new Vector2d(36, -34), (2 * Math.PI))
                            .waitSeconds(3)
                            .lineToXLinearHeading(48, (Math.PI / 4))
                            .waitSeconds(3)
//                            .splineToLinearHeading(new Vector2d(36, 24), Math.PI)
                            .splineToLinearHeading(new Pose2d(36, 24, Math.PI), 0)
                            .build());

        }
    }
}

