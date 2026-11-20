package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(-60)
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-61 , 62))
                .waitSeconds(0.5)
                .lineToY(62)
                .strafeTo(new Vector2d(40, 54))
                .waitSeconds(0.3)
                .strafeToSplineHeading(new Vector2d(50, 24), Math.toRadians(135))
                .strafeToSplineHeading(new Vector2d(62, 12), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(40, 54), Math.toRadians(90))
                .strafeTo(new Vector2d(-24 , 48))
                .waitSeconds(0.3)
                .lineToY(56)
                .lineToY(50)
                .strafeToSplineHeading(new Vector2d(40, 54), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(50, 24))
                .strafeTo(new Vector2d(58, 12))
                .waitSeconds(0.5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}