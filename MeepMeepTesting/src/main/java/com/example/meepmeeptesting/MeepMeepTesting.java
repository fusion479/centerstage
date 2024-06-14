package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(Positions.CLOSE.START)
                .lineToY(50)
                .splineTo(Positions.modifyPose(Positions.CLOSE.SPIKEMARK_RIGHT, 5, 5), Math.toRadians(220))
                .setReversed(true)
                .splineTo(Positions.CLOSE.SPIKEMARK_SETUP, Math.toRadians(90))
                .setTangent(0)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.ROBOT_LENGTH / 2, 0), 0), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}