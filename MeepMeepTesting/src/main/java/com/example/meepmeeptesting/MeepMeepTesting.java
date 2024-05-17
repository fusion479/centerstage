package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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
                .lineToY(Positions.modifyPose(Positions.FAR.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2).y)
                .lineToY(Positions.modifyPose(Positions.FAR.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2).y + 5)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.STACK_ONE, 1.3 * Constants.ROBOT_LENGTH, 0), 0), Math.toRadians(0))
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0).x)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0).x)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.ROBOT_LENGTH / 2, 0), 0), Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y, 0), Math.toRadians(180))
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0).x)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0).x)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.ROBOT_LENGTH / 2, 0), 0), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}