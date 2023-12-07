package org.firstinspires.ftc.teamcode.common.opmode.auton;


import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoConstants {
    // Starting positions
    public static double START_X = 36;
    public static double START_Y = 36;
    public static double START_HEADING = Math.toRadians(0);

    public static Pose2d BACKSTAGE_BLUE_START = new Pose2d(START_X, START_Y, START_HEADING);

    public static double SPIKE_MARK_X = 36;
    public static double SPIKE_MARK_Y = 40;

    public static Pose2d LEFT_SPIKE_MARK;
    public static Pose2d CENTER_SPIKE_MARK = new Pose2d(SPIKE_MARK_X, SPIKE_MARK_Y);
    public static Pose2d RIGHT_SPIKE_MARK;

}
