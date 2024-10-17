package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Vector2d;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(38, 60, Math.toRadians(0)))
                /*.setTangent(180)
                .splineToSplineHeading(new Pose2d(0,  31, Math.toRadians(90)), Math.toRadians(180))
                .waitSeconds(3)
                .setTangent(95)
                .splineToLinearHeading(new Pose2d(48.5, 30, Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(3)
                .setReversed(true)
                .splineTo(new Vector2d(52, 51), Math.toRadians(45))
                 */
                .strafeTo(new Vector2d(0, 25))
                .turn(Math.toRadians(90))
                .waitSeconds(3)
                .setTangent(95)
                .splineToLinearHeading(new Pose2d(48.5, 30, Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(3)
                .setReversed(true)
                .splineTo(new Vector2d(52, 51), Math.toRadians(45))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
