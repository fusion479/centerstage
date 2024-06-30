package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


class TestingConstants {
    public final double Y_OFFSET = -3.0;
    public final double X_OFFSET = 3.0;
    private final int color = 1; // 0 = blue, 1 = red

    public Pose2d reflectY(Pose2d pose) {
        return color == 1 ? new Pose2d(pose.position.x, -pose.position.y, Math.toRadians(360) - pose.heading.toDouble()) : pose;
    }

    public Vector2d reflectY(Vector2d vector) {
        return color == 1 ? new Vector2d(vector.x, -vector.y) : vector;
    }

    public double reflectY(double theta) {
        return color == 1 ? Math.toRadians(360) - theta : theta;
    }
}

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        TestingConstants utils = new TestingConstants();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 42.5, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(utils.reflectY(Positions.FAR.START))
                .lineToY(utils.reflectY(Positions.FAR.SPIKEMARK_SETUP).y)
                .splineTo(utils.reflectY(Positions.modifyPose(Positions.FAR.SPIKEMARK_LEFT, -5, 5)), utils.reflectY(Math.toRadians(320)))
                .setReversed(true)
                .splineToLinearHeading(utils.reflectY(Positions.vectorToPose(Positions.FAR.SPIKEMARK_SETUP, Math.toRadians(0))), utils.reflectY(Math.toRadians(90)))
                .lineToY(utils.reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, 0, 0)).y)
                .turnTo(0)
                .lineToX(utils.reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x)
                .lineToX(utils.reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.TILE_LENGTH, 0)).x)
                .splineToConstantHeading(utils.reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.ROBOT_LENGTH / 2, 0)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(utils.reflectY(new Vector2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y)), utils.reflectY(Math.toRadians(180)))
                .lineToX(utils.reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x, new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20, 20))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}