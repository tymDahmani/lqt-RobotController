package com.example.meepmeep_v1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class test1 {

        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(500);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();

            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -70, Math.PI / 2))
                            .lineToY(-34.5)
                            .waitSeconds(2)
                            .splineToLinearHeading(new Pose2d(49.87, -35.5, 2 * Math.PI), 0)
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(49.87, -60))
                            .setTangent(2 * Math.PI)
                            .lineToX(-61.37)
                    .build());

            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
}