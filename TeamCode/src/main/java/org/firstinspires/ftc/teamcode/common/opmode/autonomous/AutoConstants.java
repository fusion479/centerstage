package org.firstinspires.ftc.teamcode.common.opmode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;



// BLUE AUTONS POSITIONS
public final class AutoConstants {

    public static double INITIAL_FORWARD_DIST = 15.5;
    public static double MIDDLE_SPIKE_DISTANCE = 29;
    public static double ARM_LIFT_DELAY = -2.25;
    public static double PRELOAD_SCORE_DELAY = -0.5;
    public static double POST_PRELOAD_WAIT = 0.5;
    public static double STACK_PICKUP_DELAY = 1;
    public static double POST_APRILTAG_FORWARD = 6.16; // this used to be 4.5 in 2+2
    public static int APRILTAG_TIMEOUT = 2000;

    // STARTING & END POSITIONS
    public Pose2d CLOSE_START;
    public Pose2d FRONT_START;
    public Pose2d CLOSE_PARK;
    public Pose2d FRONT_PARK;

    // SPIKE & BACKDROP POSITIONS
    public Pose2d CLOSE_LEFT_SPIKE;
    public Pose2d CLOSE_RIGHT_SPIKE;
    public Pose2d FRONT_LEFT_SPIKE;
    public Pose2d FRONT_RIGHT_SPIKE;
    public Pose2d LEFT_BACKDROP_PRE;
    public Pose2d MIDDLE_BACKDROP_PRE;
    public Pose2d RIGHT_BACKDROP_PRE;
    public Pose2d LEFT_BACKDROP;
    public Pose2d MIDDLE_BACKDROP;
    public Pose2d RIGHT_BACKDROP;

    // MISC POSITIONS
    public Pose2d CLOSE_INITIAL;
    public Pose2d FRONT_INITIAL;
    public Pose2d CLOSE_MID;

    public static PoseValues V_CLOSE_START = new PoseValues(11.75, 62.7, Math.toRadians(270));
    public static PoseValues V_FRONT_START = new PoseValues(-35.25, 62.7, Math.toRadians(270));
    public static PoseValues V_CLOSE_PARK = new PoseValues(45, 57, Math.toRadians(0));
    public static PoseValues V_FRONT_PARK = new PoseValues(45, 18, Math.toRadians(0));

    // SPIKE & BACKDROP POSITIONS
    public static PoseValues V_CLOSE_LEFT_SPIKE = new PoseValues(15.5, 37, Math.toRadians(320));
    public static PoseValues V_CLOSE_RIGHT_SPIKE = new PoseValues(7.5, 37, Math.toRadians(220));
    public static PoseValues V_FRONT_LEFT_SPIKE = new PoseValues(-32, 37, Math.toRadians(320));
    public static PoseValues V_FRONT_RIGHT_SPIKE = new PoseValues(-39, 37, Math.toRadians(220));
    public static PoseValues V_LEFT_BACKDROP_PRE = new PoseValues(37.5, 42 - 2, Math.toRadians(0));
    public static PoseValues V_MIDDLE_BACKDROP_PRE = new PoseValues(39, 36 - 2, Math.toRadians(0));
    public static PoseValues V_RIGHT_BACKDROP_PRE = new PoseValues(37.5, 30 - 2, Math.toRadians(0));
    public static PoseValues V_LEFT_BACKDROP = new PoseValues(42, 42 - 2, Math.toRadians(0));
    public static PoseValues V_MIDDLE_BACKDROP = new PoseValues(42, 36 - 2, Math.toRadians(0));
    public static PoseValues V_RIGHT_BACKDROP = new PoseValues(42, 30 - 2, Math.toRadians(0));

    // MISC POSITIONS
    public static PoseValues V_CLOSE_INITIAL = new PoseValues(11.75, 44, Math.toRadians(270));
    public static PoseValues V_FRONT_INITIAL = new PoseValues(-35.25, 44, Math.toRadians(270));
    public static PoseValues V_CLOSE_MID = new PoseValues(27.5, 11, Math.toRadians(0));

    public AutoConstants() {
        INITIAL_FORWARD_DIST = 15.5;
        MIDDLE_SPIKE_DISTANCE = 29;
        ARM_LIFT_DELAY = -2.25;
        PRELOAD_SCORE_DELAY = -0.5;
        POST_PRELOAD_WAIT = 0.5;
        STACK_PICKUP_DELAY = 1;
        POST_APRILTAG_FORWARD = 6.16; // this used to be 4.5 in 2+2
        APRILTAG_TIMEOUT = 2000;

        CLOSE_START = new Pose2d(V_CLOSE_START.x, V_CLOSE_START.y, V_CLOSE_START.heading);
        FRONT_START = new Pose2d(V_FRONT_START.x, V_FRONT_START.y, V_FRONT_START.heading);
        CLOSE_PARK = new Pose2d(V_CLOSE_PARK.x, V_CLOSE_PARK.y, V_CLOSE_PARK.heading);
        FRONT_PARK = new Pose2d(V_FRONT_PARK.x, V_FRONT_PARK.y, V_FRONT_PARK.heading);

// SPIKE & BACKDROP POSITIONS
        CLOSE_LEFT_SPIKE = new Pose2d(V_CLOSE_LEFT_SPIKE.x, V_CLOSE_LEFT_SPIKE.y, V_CLOSE_LEFT_SPIKE.heading);
        CLOSE_RIGHT_SPIKE = new Pose2d(V_CLOSE_RIGHT_SPIKE.x, V_CLOSE_RIGHT_SPIKE.y, V_CLOSE_RIGHT_SPIKE.heading);
        FRONT_LEFT_SPIKE = new Pose2d(V_FRONT_LEFT_SPIKE.x, V_FRONT_LEFT_SPIKE.y, V_FRONT_LEFT_SPIKE.heading);
        FRONT_RIGHT_SPIKE = new Pose2d(V_FRONT_RIGHT_SPIKE.x, V_FRONT_RIGHT_SPIKE.y, V_FRONT_RIGHT_SPIKE.heading);
        LEFT_BACKDROP_PRE = new Pose2d(V_LEFT_BACKDROP_PRE.x, V_LEFT_BACKDROP_PRE.y, V_LEFT_BACKDROP_PRE.heading);
        MIDDLE_BACKDROP_PRE = new Pose2d(V_MIDDLE_BACKDROP_PRE.x, V_MIDDLE_BACKDROP_PRE.y, V_MIDDLE_BACKDROP_PRE.heading);
        RIGHT_BACKDROP_PRE = new Pose2d(V_RIGHT_BACKDROP_PRE.x, V_RIGHT_BACKDROP_PRE.y, V_RIGHT_BACKDROP_PRE.heading);
        LEFT_BACKDROP = new Pose2d(V_LEFT_BACKDROP.x, V_LEFT_BACKDROP.y, V_LEFT_BACKDROP.heading);
        MIDDLE_BACKDROP = new Pose2d(V_MIDDLE_BACKDROP.x, V_MIDDLE_BACKDROP.y, V_MIDDLE_BACKDROP.heading);
        RIGHT_BACKDROP = new Pose2d(V_RIGHT_BACKDROP.x, V_RIGHT_BACKDROP.y, V_RIGHT_BACKDROP.heading);

// MISC POSITIONS
        CLOSE_INITIAL = new Pose2d(V_CLOSE_INITIAL.x, V_CLOSE_INITIAL.y, V_CLOSE_INITIAL.heading);
        FRONT_INITIAL = new Pose2d(V_FRONT_INITIAL.x, V_FRONT_INITIAL.y, V_FRONT_INITIAL.heading);
        CLOSE_MID = new Pose2d(V_CLOSE_MID.x, V_CLOSE_MID.y, V_CLOSE_MID.heading);

    }

    // FOR RED AUTONS
    public static Pose2d reflectY(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), Math.toRadians(360) - pose.getHeading());
    }
}
