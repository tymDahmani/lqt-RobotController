package com.example.meepmeep1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class testing_rr_movements {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        Vector2d test1 = new Vector2d(30, 15);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(/*12, -70, 1.57*/47, -47, 0))
//                                .splineTo(new Vector2d(-24, 0), Math.PI)
//                                .waitSeconds(2)
//                                .splineToConstantHeading(new Vector2d(-58, -12), Math.PI)
//                                .waitSeconds(1)
//                                .turn(Math.PI / 2)
//                                .splineToLinearHeading(new Pose2d(10, -40, 0), 0)
//                                .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                                .splineTo(new Vector2d(0, 60), Math.PI)
                                .splineTo(new Vector2d(36, -34), (2 * Math.PI))
                                .splineToLinearHeading(new Pose2d(48, -36, (Math.PI / 4)), (Math.PI / 4))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
