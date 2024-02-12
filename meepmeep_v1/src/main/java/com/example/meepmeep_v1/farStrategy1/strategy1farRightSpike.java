package com.example.meepmeep_v1.farStrategy1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class strategy1farRightSpike {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35.5, -64.25, Math.PI / 2))
                .lineToYLinearHeading(-47, Math.PI / 3)
                .waitSeconds(4) // drop pixel
                .setTangent(Math.PI)
                .strafeToLinearHeading(new Vector2d(-52.75, -35.5), Math.PI)
                .waitSeconds(3)
                .setTangent(Math.PI / 2)
                .lineToYLinearHeading(-10.5, 2 * Math.PI)
                .setTangent(2 * Math.PI)
                .lineToXLinearHeading(47, 2 * Math.PI)
                .strafeTo(new Vector2d(47, -34.5))
                .waitSeconds(5) // drop yellow pixel in bd
                .strafeTo(new Vector2d(47, -11.5))
                .setTangent(2 * Math.PI)
                .lineToX(60) // parking
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
