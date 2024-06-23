package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

class TestingConstants {
    private final int color = 1; // 0 = blue, 1 = red
    public final double Y_OFFSET = -3.0;
    public final double X_OFFSET = 3.0;

    public Pose2d reflectY(Pose2d pose) {
        return color == 1 ? new Pose2d(pose.position.x, -pose.position.y, Math.toRadians(360) - pose.heading.toDouble()) : pose;
    }

    public Vector2d reflectY(Vector2d vector) {
        return color == 1 ? new Vector2d(vector.x, -vector.y) : vector;
    }
}
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        TestingConstants utils = new TestingConstants();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(utils.reflectY(Positions.FAR.START))
                .lineToY(utils.reflectY(Positions.modifyPose(Positions.FAR.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2)).y)
                .lineToY(utils.reflectY(Positions.modifyPose(Positions.FAR.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2 + 5)).y)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(utils.reflectY(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.STACK_ONE, 1.3 * Constants.ROBOT_LENGTH, 0), 0)), Math.toRadians(0))
                .lineToX(utils.reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x)
                .lineToX(utils.reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0)).x)
                .splineToConstantHeading(utils.reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.ROBOT_LENGTH / 2, 0)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(utils.reflectY(new Vector2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y)), Math.toRadians(180))
                .lineToX(utils.reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x)
                .lineToX(utils.reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0)).x)
                .splineToConstantHeading(utils.reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.ROBOT_LENGTH / 2, 0)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(utils.reflectY(new Vector2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y)), Math.toRadians(180))
                .lineToX(utils.reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}