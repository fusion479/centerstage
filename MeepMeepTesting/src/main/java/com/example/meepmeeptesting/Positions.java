package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Positions {
    public static Vector2d modifyPose(Vector2d pose, double dX, double dY) {
        return new Vector2d(pose.x + dX, pose.y + dY);
    }

    public static Pose2d modifyPose(Pose2d pose, double dX, double dY) {
        return new Pose2d(pose.position.x + dX, pose.position.y + dY, pose.heading.imag);
    }

    public static Pose2d vectorToPose(Vector2d vector, double heading) {
        return new Pose2d(vector.x, vector.y, heading);
    }

    public interface CLOSE {
        // START
        Pose2d START = new Pose2d(
                Constants.TILE_LENGTH / 2,
                Constants.FIELD_LENGTH - Constants.ROBOT_LENGTH / 2,
                Math.toRadians(270));


        Vector2d SPIKEMARK_SETUP = new Vector2d(Constants.TILE_LENGTH / 2, 45);

        // SPIKEMARKS
        Vector2d SPIKEMARK_MID = new Vector2d(
                START.position.x,
                Constants.FIELD_LENGTH - Constants.TILE_LENGTH * 2 - Constants.TAPE_WIDTH / 2);
        Vector2d SPIKEMARK_LEFT = new Vector2d(
                Constants.TILE_LENGTH - Constants.TAPE_WIDTH / 2,
                Constants.TILE_LENGTH + Constants.SPIKE_TAPE_LENGTH / 2);
        Vector2d SPIKEMARK_RIGHT = new Vector2d(
                0 + Constants.TAPE_WIDTH / 2,
                Constants.TILE_LENGTH + Constants.SPIKE_TAPE_LENGTH / 2);
    }

    public interface FAR {
        // START
        Pose2d START = new Pose2d(
                -Constants.TILE_LENGTH - Constants.TILE_LENGTH / 2,
                Constants.FIELD_LENGTH - Constants.ROBOT_LENGTH / 2,
                Math.toRadians(270));

        Vector2d SPIKEMARK_SETUP = new Vector2d(-Constants.TILE_LENGTH - Constants.TILE_LENGTH / 2, 45);


        Vector2d SPIKEMARK_MID = new Vector2d(
                START.position.x,
                Constants.FIELD_LENGTH - Constants.TILE_LENGTH * 2 + Constants.TAPE_WIDTH / 2);
        Vector2d SPIKEMARK_LEFT = new Vector2d(
                -Constants.TILE_LENGTH - Constants.TAPE_WIDTH / 2,
                Constants.TILE_LENGTH + Constants.SPIKE_TAPE_LENGTH / 2);
        Vector2d SPIKEMARK_RIGHT = new Vector2d(
                -Constants.TILE_LENGTH * 2 + Constants.TAPE_WIDTH / 2,
                Constants.TILE_LENGTH + Constants.SPIKE_TAPE_LENGTH / 2);
    }

    public interface GENERAL {
        Vector2d STACK_ONE = new Vector2d(
                -Constants.FIELD_LENGTH + Constants.PIXEL_LENGTH / 2 + 15,
                Constants.STACK_BETWEEN_DISTANCE + Constants.TAPE_WIDTH / 2);
        Vector2d STACK_TWO = new Vector2d(
                -Constants.FIELD_LENGTH + Constants.PIXEL_LENGTH / 2,
                2 * Constants.STACK_BETWEEN_DISTANCE + Constants.TAPE_WIDTH + Constants.TAPE_WIDTH / 2);

        // Not provided on gm2; rough estimates
        Vector2d BACKDROP_MID = new Vector2d(
                Constants.FIELD_LENGTH - Constants.BACKDROP_TO_WALL_X,
                Constants.FIELD_LENGTH - Constants.BACKDROP_TO_WALL_Y - Constants.BACKDROP_WIDTH / 2 - 1.5);
        Vector2d BACKDROP_LEFT = new Vector2d(
                Constants.FIELD_LENGTH - Constants.BACKDROP_TO_WALL_X,
                Constants.FIELD_LENGTH - Constants.BACKDROP_TO_WALL_Y - Constants.BACKDROP_WIDTH / 2 - 1.5 + 6);
        Vector2d BACKDROP_RIGHT = new Vector2d(
                Constants.FIELD_LENGTH - Constants.BACKDROP_TO_WALL_X,
                Constants.FIELD_LENGTH - Constants.BACKDROP_TO_WALL_Y - Constants.BACKDROP_WIDTH / 2 - 1.5 - 6);
    }
}
