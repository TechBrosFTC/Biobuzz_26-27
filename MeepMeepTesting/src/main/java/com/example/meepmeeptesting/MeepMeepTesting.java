package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Image img = null;
        try {
            // Coloque o caminho completo ou relativo até a sua imagem (PNG ou JPG)
            img = ImageIO.read(new File("MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\834lq6c925ph1.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }


        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, 12, Math.toRadians(90)))
                .lineToY( 62)
                .lineToY(59)
                .splineToLinearHeading(new Pose2d(new Vector2d(38, 38), Math.toRadians(180)), Math.toRadians(-45))
                .strafeTo(new Vector2d(56, 38))
                .strafeToSplineHeading(new Vector2d(-24, 52), Math.toRadians(90))
                .strafeTo(new Vector2d(-24, 56))
                .strafeTo(new Vector2d(-24, 52))
                .strafeToSplineHeading(new Vector2d(56, 38), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(52, 24), Math.toRadians(0))
                .strafeTo(new Vector2d(56, 24))
                .strafeToSplineHeading(new Vector2d( 48, 61), Math.toRadians(270))
                .build());


        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}