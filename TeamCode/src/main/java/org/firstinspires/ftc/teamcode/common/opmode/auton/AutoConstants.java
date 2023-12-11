package org.firstinspires.ftc.teamcode.common.opmode.auton;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class AutoConstants {
    // Starting positions
    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_HEADING = Math.toRadians(0);

    public static Pose2d BACKSTAGE_BLUE_START = new Pose2d(START_X, START_Y, START_HEADING);

    public static double CENTER_SPIKE_MARK_X = 20;
    public static double CENTER_SPIKE_MARK_Y = 20;

    public static Vector2d LEFT_SPIKE_MARK = new Vector2d(CENTER_SPIKE_MARK_X + 6, CENTER_SPIKE_MARK_Y - 6);
    public static Pose2d CENTER_SPIKE_MARK = new Pose2d(CENTER_SPIKE_MARK_X, CENTER_SPIKE_MARK_Y);
    public static Vector2d RIGHT_SPIKE_MARK = new Vector2d(CENTER_SPIKE_MARK_X + 6, CENTER_SPIKE_MARK_Y + 6);

}
