package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;

public class Positions {
    public interface CLOSE {
        Pose2d START = new Pose2d(0, 0, 0);
        Pose2d BACKDROP_MID = new Pose2d(0, 0, 0);
        Pose2d BACKDROP_LEFT = new Pose2d(0, 0, 0);
        Pose2d BACKDROP_RIGHT = new Pose2d(0, 0, 0);
    }

    public interface FAR {
        Pose2d START = new Pose2d(0, 0, 0);
        Pose2d BACKDROP_MID = new Pose2d(0, 0, 0);
        Pose2d BACKDROP_LEFT = new Pose2d(0, 0, 0);
        Pose2d BACKDROP_RIGHT = new Pose2d(0, 0, 0);
    }
}
