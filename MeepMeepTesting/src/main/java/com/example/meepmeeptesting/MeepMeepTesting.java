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

        myBot.runAction(myBot.getDrive().actionBuilder(Positions.FAR.START)
                .lineToY(Positions.FAR.SPIKEMARK_SETUP.y)
                .splineTo(Positions.modifyPose(Positions.FAR.SPIKEMARK_RIGHT, 5, 5), Math.toRadians(220))
                .setReversed(true)
                .splineTo(Positions.FAR.SPIKEMARK_SETUP, Math.toRadians(90))
                .lineToY(Positions.GENERAL.STACK_ONE.y)
                .turnTo(0)
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0).x)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.TILE_LENGTH, 0).x)
                .splineToConstantHeading(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.ROBOT_LENGTH / 2, 0), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}