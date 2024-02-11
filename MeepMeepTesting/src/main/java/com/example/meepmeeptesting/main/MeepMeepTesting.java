package com.example.meepmeeptesting.main;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static Pose2d BLUE_BACKSTAGE_START = new Pose2d(12, 62.7, Math.toRadians(270));
    public static Pose2d RED_BACKSTAGE_START = new Pose2d(12, -62.7, Math.toRadians(90));
    public static Pose2d BLUE_FRONT_START = new Pose2d(-36, 62.7, Math.toRadians(270));
    public static Pose2d RED_FRONT_START = new Pose2d(-36, -62.7, Math.toRadians(90));

    public static double INITIAL_FORWARD_DIST = 14;
    public static double MIDDLE_SPIKE_DISTANCE = 27;
    public static double RB_PRELOAD_FORWARD_DIST = 37;

    public static double armLiftDelay = -1.5;
    public static double preloadScoreDelay = .5;
    public static double postPreloadWait = 2;
    public static Pose2d B_PARK = new Pose2d(45, 60, Math.toRadians(0));
    public static Pose2d R_PARK = new Pose2d(45, -60, Math.toRadians(0));

    public static Pose2d RB_L_SPIKE = new Pose2d(8, -34, Math.toRadians(140));
    public static Pose2d RB_R_SPIKE = new Pose2d(16, -34, Math.toRadians(50));
    public static Pose2d RB_L_BACKDROP = new Pose2d(54, -29.5, Math.toRadians(0));
    public static Pose2d RB_M_BACKDROP = new Pose2d(54, -35, Math.toRadians(0));
    public static Pose2d RB_R_BACKDROP = new Pose2d(54, -42.5, Math.toRadians(0));

    public static Pose2d BB_L_SPIKE = new Pose2d(16, 38, Math.toRadians(320));
    public static Pose2d BB_R_SPIKE = new Pose2d(8, 38, Math.toRadians(220));
    public static Pose2d BB_L_BACKDROP = new Pose2d(54, 41.5, Math.toRadians(0));
    public static Pose2d BB_M_BACKDROP = new Pose2d(54, 34.5, Math.toRadians(0));
    public static Pose2d BB_R_BACKDROP = new Pose2d(54, 29.5, Math.toRadians(0));

    public static Pose2d RF_L_SPIKE = new Pose2d(-40, -34, Math.toRadians(140));
    public static Pose2d RF_R_SPIKE = new Pose2d(-32, -34, Math.toRadians(50));
    public static Pose2d RF_L_BACKDROP = new Pose2d(54, -28, Math.toRadians(0));
    public static Pose2d RF_M_BACKDROP = new Pose2d(54, -35, Math.toRadians(0));
    public static Pose2d RF_R_BACKDROP = new Pose2d(54, -42, Math.toRadians(0));

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
//                                .forward(14)
//                                .setTangent(Math.toRadians(90))
//                                .splineToLinearHeading(RF_L_SPIKE, RF_L_SPIKE.getHeading())
//                                .setTangent(Math.toRadians(320))
//                                .splineToLinearHeading(new Pose2d(-34, -38, Math.toRadians(90)), Math.toRadians(270))
//                                .lineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(90)))
//                                .turn(Math.toRadians(-90))
//                                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
//                                .splineToLinearHeading(RF_L_BACKDROP, Math.toRadians(0))

                                .forward(MIDDLE_SPIKE_DISTANCE)
                                .back(5)
                                // END OF SPIKE MARK
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-52, -24, Math.toRadians(90)), Math.toRadians(90))
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(0)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
                                .splineToLinearHeading(RF_M_BACKDROP, Math.toRadians(0))

//                                .forward(14)
//                                .setTangent(Math.toRadians(90))
//                                .splineToLinearHeading(RF_R_SPIKE, Math.toRadians(50))
//                                .setTangent(Math.toRadians(230))
//                                .splineToLinearHeading(new Pose2d(-40, -38, Math.toRadians(0)), Math.toRadians(90))
//                                .lineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
//                                .splineToLinearHeading(RF_R_BACKDROP, Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}