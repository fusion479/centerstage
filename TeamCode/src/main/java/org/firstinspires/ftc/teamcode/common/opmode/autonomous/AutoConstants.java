package org.firstinspires.ftc.teamcode.common.opmode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;

// BLUE AUTONS POSITIONS
public final class AutoConstants {
    // TIMERS & GENERAL DISTANCES
    public static double INITIAL_FORWARD_DIST = 14;
    public static double MIDDLE_SPIKE_DISTANCE = 28.5;
    public static double ARM_LIFT_DELAY = -2.25;
    public static double PRELOAD_SCORE_DELAY = .5;
    public static double POST_PRELOAD_WAIT = 2;

    // STARTING & END POSITIONS
    public static Pose2d CLOSE_START = new Pose2d(11.75, 62.7, Math.toRadians(270));
    public static Pose2d FRONT_START = new Pose2d(-35.25, 62.7, Math.toRadians(270));
    public static Pose2d CLOSE_PARK = new Pose2d(45, 57, Math.toRadians(0));
    public static Pose2d FRONT_PARK = new Pose2d(45, 18, Math.toRadians(0));

    // SPIKE & BACKDROP POSITIONS
    public static Pose2d CLOSE_LEFT_SPIKE = new Pose2d(16, 37, Math.toRadians(320));
    public static Pose2d CLOSE_RIGHT_SPIKE = new Pose2d(8, 37, Math.toRadians(220));
    public static Pose2d FRONT_LEFT_SPIKE = new Pose2d(-32, 37, Math.toRadians(320));
    public static Pose2d FRONT_RIGHT_SPIKE = new Pose2d(-40, 37, Math.toRadians(220));
    public static Pose2d CLOSE_LEFT_BACKDROP = new Pose2d(43, 42 - 2, Math.toRadians(0));
    public static Pose2d CLOSE_MIDDLE_BACKDROP = new Pose2d(43, 36 - 2, Math.toRadians(0));
    public static Pose2d CLOSE_RIGHT_BACKDROP = new Pose2d(43, 30 - 2, Math.toRadians(0));
    public static Pose2d FRONT_LEFT_BACKDROP = new Pose2d(56, 41 - 2, Math.toRadians(0));
    public static Pose2d FRONT_MIDDLE_BACKDROP = new Pose2d(56, 35 + 1, Math.toRadians(0));
    public static Pose2d FRONT_RIGHT_BACKDROP = new Pose2d(56, 29 - 2, Math.toRadians(0));

    // MISC POSITIONS
    public static Pose2d CLOSE_INITIAL = new Pose2d(11.75, 44, Math.toRadians(270));
    public static Pose2d FRONT_INITIAL = new Pose2d(-35.25, 44, Math.toRadians(270));
    public static Pose2d CLOSE_MID = new Pose2d(27.5, 12, Math.toRadians(0));

    public static Pose2d CLOSE_POST_PRELOAD = new Pose2d(16, 36, Math.toRadians(0));

    public static Pose2d STACK_1 = new Pose2d(-50, 36, Math.toRadians(0));

    // FOR RED AUTONS
    public static Pose2d reflectY(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), Math.toRadians(360) - pose.getHeading());
    }
}
