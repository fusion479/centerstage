package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.sun.org.apache.bcel.internal.Const;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(Positions.FAR.START)
                .lineToY(50)
                .splineTo(Positions.modifyPose(Positions.FAR.SPIKEMARK_LEFT, -5, 5), Math.toRadians(320))
                .setReversed(true)
                .splineToLinearHeading(Positions.vectorToPose(Positions.FAR.SPIKEMARK_SETUP, Math.toRadians(0)), Math.toRadians(90))
                .lineToY(Positions.GENERAL.STACK_ONE.y)
                .turnTo(0)
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH/2, 0).x)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, - 2 * Constants.ROBOT_LENGTH, 0).x)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.ROBOT_LENGTH / 2 - 3, 0), 0), Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(180)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.CLOSE.SPIKEMARK_MID, 20, -14), 0), 181)
                .strafeTo(new Vector2d(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH/2, 0).x, Positions.GENERAL.STACK_ONE.y))
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -2 * Constants.ROBOT_LENGTH, 0).x)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.ROBOT_LENGTH/2 - 3, 0), 0), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}