package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Positions {
    public interface CLOSE {
        // START
        Pose2d START = new Pose2d(
                Constants.TILE_LENGTH / 2,
                Constants.FIELD_LENGTH - Constants.ROBOT_LENGTH / 2,
                Math.toRadians(270));

        // SPIKEMARKS
        Vector2d SPIKEMARK_MID = new Vector2d(
                START.position.x,
                Constants.FIELD_LENGTH - Constants.TILE_LENGTH * 2 + Constants.SPIKE_TAPE_WIDTH / 2);
        Vector2d SPIKEMARK_LEFT = new Vector2d(
                Constants.TILE_LENGTH - Constants.SPIKE_TAPE_WIDTH / 2,
                Constants.TILE_LENGTH + Constants.SPIKE_TAPE_LENGTH / 2);
        Vector2d SPIKEMARK_RIGHT = new Vector2d(
                0 + Constants.SPIKE_TAPE_WIDTH / 2,
                Constants.TILE_LENGTH + Constants.SPIKE_TAPE_LENGTH / 2);
    }

    public interface FAR {
        // START
        Pose2d START = new Pose2d(
                -Constants.TILE_LENGTH - Constants.TILE_LENGTH / 2,
                Constants.FIELD_LENGTH - Constants.ROBOT_LENGTH / 2,
                Math.toRadians(270));

        Vector2d SPIKEMARK_MID = new Vector2d(
                START.position.x,
                Constants.FIELD_LENGTH - Constants.TILE_LENGTH * 2 + Constants.SPIKE_TAPE_WIDTH / 2);
        Vector2d SPIKEMARK_LEFT = new Vector2d(
                -Constants.TILE_LENGTH - Constants.SPIKE_TAPE_WIDTH / 2,
                Constants.TILE_LENGTH + Constants.SPIKE_TAPE_LENGTH / 2);
        Vector2d SPIKEMARK_RIGHT = new Vector2d(
                -Constants.TILE_LENGTH * 2 + Constants.SPIKE_TAPE_WIDTH / 2,
                Constants.TILE_LENGTH + Constants.SPIKE_TAPE_LENGTH / 2);
    }

    public interface GENERAL {
        Pose2d BACKDROP_MID = new Pose2d(0, 0, 0);
        Pose2d BACKDROP_LEFT = new Pose2d(0, 0, 0);
        Pose2d BACKDROP_RIGHT = new Pose2d(0, 0, 0);
    }
}
