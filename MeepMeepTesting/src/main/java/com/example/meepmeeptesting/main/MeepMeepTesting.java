package com.example.meepmeeptesting.main;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static Pose2d BLUE_BACKSTAGE_START = new Pose2d(12, 62.7, Math.toRadians(270));
    public static Pose2d RED_BACKSTAGE_START = new Pose2d(12, -62.7, Math.toRadians(90));
    public static Pose2d BLUE_FRONT_START = new Pose2d(-36, 62.7, Math.toRadians(270));
    public static Pose2d RED_FRONT_START = new Pose2d(-36, -62.7, Math.toRadians(90));

    public static double INITIAL_FORWARD_DIST = 14;
    public static double MIDDLE_SPIKE_DISTANCE = 27.5;
    public static double RB_PRELOAD_FORWARD_DIST = 37;

    public static double armLiftDelay = -1;
    public static double preloadScoreDelay = .5;
    public static double postPreloadWait = 2;

    public static Pose2d RB_L_SPIKE = new Pose2d(8, -34, Math.toRadians(140));
    public static Pose2d RB_R_SPIKE = new Pose2d(16, -34, Math.toRadians(50));
    public static Pose2d RB_L_BACKDROP = new Pose2d(52, -30, Math.toRadians(0));
    public static Pose2d RB_M_BACKDROP = new Pose2d(52, -36, Math.toRadians(0));
    public static Pose2d RB_R_BACKDROP = new Pose2d(52, -43, Math.toRadians(0));

    public static Pose2d BB_L_SPIKE = new Pose2d(16, 34, Math.toRadians(320));
    public static Pose2d BB_R_SPIKE = new Pose2d(8, 34, Math.toRadians(220));
    public static Pose2d BB_L_BACKDROP = new Pose2d(52, 41, Math.toRadians(0));
    public static Pose2d BB_M_BACKDROP = new Pose2d(51.5, 38, Math.toRadians(0));
    public static Pose2d BB_R_BACKDROP = new Pose2d(52, 29, Math.toRadians(0));

    public static Pose2d RF_L_SPIKE = new Pose2d(-40, -34, Math.toRadians(140));
    public static Pose2d RF_R_SPIKE = new Pose2d(-32, -34, Math.toRadians(50));
    public static Pose2d RF_L_BACKDROP = new Pose2d(52, -30, Math.toRadians(0));
    public static Pose2d RF_M_BACKDROP = new Pose2d(52, -36, Math.toRadians(0));
    public static Pose2d RF_R_BACKDROP = new Pose2d(52, -42, Math.toRadians(0));

    public static Pose2d BF_L_SPIKE = new Pose2d(-32, 34, Math.toRadians(320));
    public static Pose2d BF_R_SPIKE = new Pose2d(-40, 34, Math.toRadians(220));
    public static Pose2d BF_L_BACKDROP = new Pose2d(52, 42, Math.toRadians(0));
    public static Pose2d BF_M_BACKDROP = new Pose2d(52, 36, Math.toRadians(0));
    public static Pose2d BF_R_BACKDROP = new Pose2d(52, 30, Math.toRadians(0));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, 5, toRadians(60), 15.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_FRONT_START)
                                .forward(MIDDLE_SPIKE_DISTANCE)
                                .back(5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}