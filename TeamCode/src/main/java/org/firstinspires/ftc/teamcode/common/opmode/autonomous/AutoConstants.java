package org.firstinspires.ftc.teamcode.common.opmode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoConstants {

    public static Pose2d BLUE_BACKSTAGE_START = new Pose2d(12, 62.7, Math.toRadians(270));
    public static Pose2d RED_BACKSTAGE_START = new Pose2d(12, -62.7, Math.toRadians(90));
    public static Pose2d BLUE_FRONT_START = new Pose2d(-36, 62.7, Math.toRadians(270));
    public static Pose2d RED_FRONT_START = new Pose2d(-36, -62.7, Math.toRadians(90));

    public static double INITIAL_FORWARD_DIST = 14;
    public static double MIDDLE_SPIKE_DISTANCE = 26.5;
    public static double RB_PRELOAD_FORWARD_DIST = 38;

    public static double armLiftDelay = -1;
    public static double preloadScoreDelay = .5;
    public static double postPreloadWait = 2;

    public static Pose2d RB_L_SPIKE = new Pose2d(8, -39, Math.toRadians(140));
    public static Pose2d RB_R_SPIKE = new Pose2d(16, -39, Math.toRadians(50));
    public static Pose2d RB_L_BACKDROP = new Pose2d(12, -30, Math.toRadians(0));
    public static Pose2d RB_M_BACKDROP = new Pose2d(12, -36, Math.toRadians(0));
    public static Pose2d RB_R_BACKDROP = new Pose2d(12, -42, Math.toRadians(0));

    public static Pose2d BB_L_SPIKE = new Pose2d(16, 39, Math.toRadians(320));
    public static Pose2d BB_R_SPIKE = new Pose2d(8, 39, Math.toRadians(220));
    public static Pose2d BB_L_BACKDROP = new Pose2d(12, 42, Math.toRadians(0));
    public static Pose2d BB_M_BACKDROP = new Pose2d(12, 36, Math.toRadians(0));
    public static Pose2d BB_R_BACKDROP = new Pose2d(12, 30, Math.toRadians(0));

    public static Pose2d RF_L_SPIKE = new Pose2d(-40, -34, Math.toRadians(140));
    public static Pose2d RF_R_SPIKE = new Pose2d(-32, -34, Math.toRadians(50));
    public static Pose2d RF_L_BACKDROP = new Pose2d(47, -30, Math.toRadians(0));
    public static Pose2d RF_M_BACKDROP = new Pose2d(47, -36, Math.toRadians(0));
    public static Pose2d RF_R_BACKDROP = new Pose2d(47, -42, Math.toRadians(0));

    public static Pose2d BF_L_SPIKE = new Pose2d(-32, 34, Math.toRadians(320));
    public static Pose2d BF_R_SPIKE = new Pose2d(-40, 34, Math.toRadians(220));
    public static Pose2d BF_L_BACKDROP = new Pose2d(47, 42, Math.toRadians(0));
    public static Pose2d BF_M_BACKDROP = new Pose2d(47, 36, Math.toRadians(0));
    public static Pose2d BF_R_BACKDROP = new Pose2d(47, 30, Math.toRadians(0));
}
