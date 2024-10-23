package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61, Math.toRadians(90)))

//                        .lineToXSplineHeading(11, Math.toRadians(0))
//                        .setTangent(Math.toRadians(0))
//                        .turn(Math.toRadians(45))
//                        .waitSeconds(0.1)
//
//                        .lineToYSplineHeading(36, Math.toRadians(180))
//                        .setTangent(Math.toRadians(180))
//                        .lineToXSplineHeading(-24, Math.toRadians(-90))
//                        .setTangent(Math.toRadians(-90))
//                        .lineToYSplineHeading(0, Math.toRadians(0))
//                        .setTangent(Math.toRadians(0))
//                        .lineToX(0)
//                .lineToY(-40)
//                .turn(Math.toRadians(90))
//                 .lineToX(-54)
//                .turn(Math.toRadians(90))

                .setTangent(Math.toRadians(90+70))
                .lineToXSplineHeading(-44, Math.toRadians(225))
                .setTangent(Math.toRadians(225))
                .lineToY(-54)
                .turnTo(Math.toRadians(270))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}