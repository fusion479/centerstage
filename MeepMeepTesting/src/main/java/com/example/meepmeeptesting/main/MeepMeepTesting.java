package com.example.meepmeeptesting.main;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

class AutoConstants {
    // TIMERS & GENERAL DISTANCES
    public static double INITIAL_FORWARD_DIST = 15.5;
    public static double MIDDLE_SPIKE_DISTANCE = 28.5;
    public static double ARM_LIFT_DELAY = -2.25;
    public static double PRELOAD_SCORE_DELAY = 1.5;
    public static double POST_PRELOAD_WAIT = 0.75;
    public static double STACK_PICKUP_DELAY = 0.5;
    public static double POST_APRILTAG_FORWARD = 4.5;

    // STARTING & END POSITIONS
    public static Pose2d CLOSE_START = new Pose2d(11.75, 62.7, Math.toRadians(270));
    public static Pose2d FRONT_START = new Pose2d(-35.25, 62.7, Math.toRadians(270));
    public static Pose2d CLOSE_PARK = new Pose2d(45, 57, Math.toRadians(0));
    public static Pose2d FRONT_PARK = new Pose2d(45, 18, Math.toRadians(0));

    // SPIKE & BACKDROP POSITIONS
    public static Pose2d CLOSE_LEFT_SPIKE = new Pose2d(17, 37, Math.toRadians(320));
    public static Pose2d CLOSE_RIGHT_SPIKE = new Pose2d(9, 37, Math.toRadians(220));
    public static Pose2d FRONT_LEFT_SPIKE = new Pose2d(-33, 37, Math.toRadians(320));
    public static Pose2d FRONT_RIGHT_SPIKE = new Pose2d(-39, 37, Math.toRadians(220));
    public static Pose2d LEFT_BACKDROP = new Pose2d(37.5, 42 - 2, Math.toRadians(0));
    public static Pose2d MIDDLE_BACKDROP = new Pose2d(39, 36 - 2, Math.toRadians(0));
    public static Pose2d RIGHT_BACKDROP = new Pose2d(37.5, 30 - 2, Math.toRadians(0));

    // MISC POSITIONS
    public static Pose2d CLOSE_INITIAL = new Pose2d(11.75, 44, Math.toRadians(270));
    public static Pose2d FRONT_INITIAL = new Pose2d(-35.25, 44, Math.toRadians(270));
    public static Pose2d CLOSE_MID = new Pose2d(27.5, 12, Math.toRadians(0));

    // FOR 2 + 2 AUTONS
    public static Pose2d BACKDROP_TRUSS_ENTRANCE = new Pose2d(20, 58, Math.toRadians(0));
    public static Pose2d WING_TRUSS_ENTRANCE = new Pose2d(-40, 58, Math.toRadians(0));
    public static Pose2d STACK_1 = new Pose2d(-55, 36, Math.toRadians(0));

    // FOR RED AUTONS
    public static Pose2d reflectY(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), Math.toRadians(360) - pose.getHeading());
    }
}

public class MeepMeepTesting extends AutoConstants {
    private static final double STARTING_ANGLE = 120; // + makes angle narrower (hits truss)     - makes angle steeper (hits wall)
    private static final double ENDING_ANGLE = 240;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, 5, toRadians(60), 15.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(MIDDLE_BACKDROP)
                        .setTangent(Math.toRadians(STARTING_ANGLE))
                        .splineToConstantHeading(new Vector2d(15, 59), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-30, 59), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-55, 36), Math.toRadians(ENDING_ANGLE))
                        .setTangent(Math.toRadians(ENDING_ANGLE - 180))
                        .splineToConstantHeading(new Vector2d(-30, 59), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(15, 59), Math.toRadians(0))
                        .splineToConstantHeading(MIDDLE_BACKDROP.vec(), Math.toRadians(STARTING_ANGLE + 180))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}