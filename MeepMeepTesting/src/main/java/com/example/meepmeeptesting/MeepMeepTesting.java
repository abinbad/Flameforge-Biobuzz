package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.Image;
import java.io.IOException;
import java.io.InputStream;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 70, 1.5*Math.PI, 1.5*Math.PI, 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .setDimensions(16.75, 17.5)
                .setDriveTrainType(DriveTrainType.TANK)
                .build();


         myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-62, 12, Math.toRadians(180)))
//               .stopAndAdd(shoot.close_fire())
                .lineToX(-36)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(-32, 42), Math.toRadians(90), new TranslationalVelConstraint(20))
                .splineTo(new Vector2d(-46,59.5), Math.toRadians(180))
//                 .stopAndAdd(intake.startIntake())
                .lineToX(-61)
                .waitSeconds(.75)
//                 .stopAndAdd(intake.stopIntake())
                .setReversed(true)
                .splineTo(new Vector2d(0,48), Math.toRadians(0))
                .splineTo(new Vector2d(46, 52), Math.toRadians(50))
                .turn(Math.toRadians(180))
//                 .stopAndAdd(shoot.soft_fire())
                         .build()
         );

         RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                 .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                 .setColorScheme(new ColorSchemeRedLight())
                 .build();

//        bot2.runAction(bot2.getDrive().actionBuilder(new Pose2d(0, 0, 0))
//                //.lineToX(61)
//                .build()
//        );

        Image img;
        try (InputStream in = MeepMeepTesting.class.getResourceAsStream("/Juice-BIOBUZZ-Dark.png")) {
            if (in == null) {
                throw new IOException("Image not found on classpath.");
            }
            img = ImageIO.read(in);
            if (img == null) {
                throw new IOException("Found the file but no ImageIO reader could decode it.");
            }
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .addEntity(bot2)
                .start();
    }
}