package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.meepmeeptesting.Constants;
import com.example.meepmeeptesting.Positions;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;

public class Trajectories {
    private final Camera.Color color;
    private final MecanumDrive drive;

    public Trajectories(Camera.Color color, MecanumDrive drive) {
        this.color = color;
        this.drive = drive;
    }

    public Pose2d reflectY(Pose2d pose) {
        return this.color == Camera.Color.RED ? new Pose2d(pose.position.x, -pose.position.y, Math.toRadians(360) - pose.heading.imag) : pose;
    }

    public Vector2d reflectY(Vector2d vector) {
        return this.color == Camera.Color.RED ? new Vector2d(vector.x, -vector.y) : vector;
    }

    public class Far {
        // SPIKEMARK
        public double Y_OFFSET = -3.0;
        public double X_OFFSET = 3.0;

        public Action MID_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(reflectY(Positions.modifyPose(Positions.FAR.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2)).y)
                .lineToY(reflectY(Positions.modifyPose(Positions.FAR.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2)).y + 5)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(reflectY(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.STACK_ONE, 1.3 * Constants.ROBOT_LENGTH, Y_OFFSET), 0)), Math.toRadians(0))
                .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2 + X_OFFSET, 0)).x)
                .build();

        public Action RIGHT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(reflectY(Positions.FAR.SPIKEMARK_SETUP).y)
                .splineTo(reflectY(Positions.modifyPose(Positions.FAR.SPIKEMARK_RIGHT, 5, 5)), Math.toRadians(220))
                .setReversed(true)
                .splineTo(reflectY(Positions.FAR.SPIKEMARK_SETUP), Math.toRadians(90))
                .lineToY(reflectY(Positions.GENERAL.STACK_ONE).y)
                .turnTo(0)
                .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x)
                .build();

        public Action LEFT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(reflectY(Positions.FAR.SPIKEMARK_SETUP).y)
                .splineTo(reflectY(Positions.modifyPose(Positions.FAR.SPIKEMARK_LEFT, -5, 5)), Math.toRadians(320))
                .setReversed(true)
                .splineToLinearHeading(reflectY(Positions.vectorToPose(Positions.FAR.SPIKEMARK_SETUP, Math.toRadians(0))), Math.toRadians(90))
                .lineToY(reflectY(Positions.GENERAL.STACK_ONE).y)
                .turnTo(0)
                .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x)
                .build();

        // PARK
        public Action getPark() {
            return drive.actionBuilder(drive.pose)
                    .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.ROBOT_LENGTH, 0)).x)
                    .strafeTo(reflectY(Positions.FAR.PARK_FAR))
                    .build();
        }
    }

    public class General {
        public double Y_OFFSET = 1.5;
        public double X_OFFSET = 0;

        public Action MID_BACKDROP_TO_STACK = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(reflectY(new Vector2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y)), Math.toRadians(180))
                .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x)
                .build();

        public Action STACK_TO_MID_BACKDROP = drive.actionBuilder(drive.pose)
                .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0)).x)
                .splineToConstantHeading(reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.ROBOT_LENGTH / 2 + X_OFFSET, Y_OFFSET)), Math.toRadians(0))
                .build();

        public Action RIGHT_BACKDROP_TO_STACK = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(reflectY(new Vector2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y)), Math.toRadians(180))
                .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x)
                .build();

        public Action STACK_TO_RIGHT_BACKDROP = drive.actionBuilder(drive.pose)
                .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.TILE_LENGTH, 0)).x)
                .splineToConstantHeading(reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.ROBOT_LENGTH / 2, 0)), Math.toRadians(0))
                .build();

        public Action LEFT_BACKDROP_TO_STACK = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(reflectY(new Vector2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y)), Math.toRadians(180))
                .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0)).x)
                .build();

        public Action STACK_TO_LEFT_BACKDROP = drive.actionBuilder(drive.pose)
                .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.TILE_LENGTH, 0)).x)
                .splineToConstantHeading(reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.ROBOT_LENGTH / 2, 0)), Math.toRadians(0))
                .build();

    }

    public class Close {
        public double Y_OFFSET = 7.0;
        public double X_OFFSET = 2.0;

        // SPIKEMARK
        public Action MID_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(reflectY(Positions.modifyPose(Positions.CLOSE.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2)).y)
                .lineToY(reflectY(Positions.modifyPose(Positions.CLOSE.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2)).y + 5)
                .setTangent(0)
                .splineToLinearHeading(
                        reflectY(Positions.modifyPose(
                                Positions.vectorToPose(Positions.GENERAL.BACKDROP_MID, Math.toRadians(0)),
                                -Constants.ROBOT_LENGTH / 2 + X_OFFSET, Y_OFFSET)),
                        Math.toRadians(0))
                .build();

        public Action RIGHT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(50)
                .splineTo(reflectY(Positions.modifyPose(Positions.CLOSE.SPIKEMARK_RIGHT, 5, 5)), Math.toRadians(220))
                .setReversed(true)
                .splineTo(reflectY(Positions.CLOSE.SPIKEMARK_SETUP), Math.toRadians(90))
                .setTangent(0)
                .splineToLinearHeading(reflectY(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.ROBOT_LENGTH / 2 + X_OFFSET, Y_OFFSET - 2), 0)), Math.toRadians(0))
                .build();

        public Action LEFT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(50)
                .splineTo(
                        reflectY(Positions.modifyPose(Positions.CLOSE.SPIKEMARK_LEFT, -5, 5)),
                        Math.toRadians(320))
                .setReversed(true)
                .splineToLinearHeading(
                        reflectY(Positions.vectorToPose(Positions.CLOSE.SPIKEMARK_SETUP, Math.toRadians(0))),
                        Math.toRadians(90))
                .setTangent(0)
                .splineToLinearHeading(reflectY(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.ROBOT_LENGTH / 2, Y_OFFSET), 0)), Math.toRadians(0))
                .build();

        public Action getPark() {
            return drive.actionBuilder(drive.pose)
                    .lineToX(reflectY(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.ROBOT_LENGTH, 0)).x)
                    .strafeTo(reflectY(Positions.modifyPose(Positions.CLOSE.PARK_CLOSE, 0, Y_OFFSET)))
                    .build();
        }
    }
}