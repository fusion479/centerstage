package org.firstinspires.ftc.teamcode.common.opmode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoConstants {

    public static Pose2d BLUE_BACKSTAGE_START = new Pose2d(12, 63.25, Math.toRadians(270));
    public static Pose2d RED_BACKSTAGE_START = new Pose2d(12, -63.25, Math.toRadians(90));
    public static Pose2d BLUE_FRONT_START = new Pose2d(-36, 63.25, Math.toRadians(270));
    public static Pose2d RED_FRONT_START = new Pose2d(-36, -63.25, Math.toRadians(90));

    public static double MIDDLE_SPIKE_DISTANCE = 26.5;
    public static double RED_BACKSTAGE_PRELOAD_FORWARD_DIST = 37.5;

    public static double armLiftDelay = -1;
    public static double preloadScoreDelay = .5;
    public static double postPreloadWait = 2;

}
