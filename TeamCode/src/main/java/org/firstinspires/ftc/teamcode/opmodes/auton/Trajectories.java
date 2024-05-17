package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;

public class Trajectories {
    private final Camera.Color color;
    private final MecanumDrive drive;

    public Trajectories(Camera.Color color, Drivetrain drivetrain) {
        this.color = color;
        this.drive = drivetrain.drive;
    }

    public Pose2d reflectY(Pose2d pose) {
        return this.color == Camera.Color.RED ? new Pose2d(pose.position.x, -pose.position.y, Math.toRadians(360) - pose.heading.imag) : pose;
    }

    public class Far {
        // SPIKEMARK
        public Action MID_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(Positions.modifyPose(Positions.FAR.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2).y)
                .lineToY(Positions.modifyPose(Positions.FAR.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2).y + 5)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.STACK_ONE, 1.3 * Constants.ROBOT_LENGTH, 0), 0), Math.toRadians(0))
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0).x)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0).x)
                .splineToConstantHeading(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.ROBOT_LENGTH / 2, 0), 0)
                .build();
        public Action RIGHT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(Positions.FAR.SPIKEMARK_SETUP.y)
                .splineTo(Positions.modifyPose(Positions.FAR.SPIKEMARK_RIGHT, 5, 5), Math.toRadians(220))
                .setReversed(true)
                .splineTo(Positions.FAR.SPIKEMARK_SETUP, Math.toRadians(90))
                .lineToY(Positions.GENERAL.STACK_ONE.y)
                .turnTo(0)
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0).x)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.TILE_LENGTH, 0).x)
                .splineToConstantHeading(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.ROBOT_LENGTH / 2, 0), 0)
                .build();
        public Action LEFT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(Positions.FAR.SPIKEMARK_SETUP.y)
                .splineTo(Positions.modifyPose(Positions.FAR.SPIKEMARK_LEFT, -5, 5), Math.toRadians(320))
                .setReversed(true)
                .splineToLinearHeading(Positions.vectorToPose(Positions.FAR.SPIKEMARK_SETUP, Math.toRadians(0)), Math.toRadians(90))
                .lineToY(Positions.GENERAL.STACK_ONE.y)
                .turnTo(0)
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0).x)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.TILE_LENGTH, 0).x)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.ROBOT_LENGTH / 2, 0), 0), Math.toRadians(0))
                .build();

        // PARK
        public Action PARK = drive.actionBuilder(drive.pose)
                .build();
    }

    public class General {
        public Action MID_BACKDROP_TO_STACK = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y, 0), Math.toRadians(180))
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0).x)
                .build();

        public Action STACK_TO_MID_BACKDROP = drive.actionBuilder(drive.pose)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.TILE_LENGTH, 0).x)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_MID, -Constants.ROBOT_LENGTH / 2, 0), 0), Math.toRadians(0))
                .build();

        public Action RIGHT_BACKDROP_TO_STACK = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y, 0), Math.toRadians(180))
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0).x)
                .build();

        public Action STACK_TO_RIGHT_BACKDROP = drive.actionBuilder(drive.pose)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.TILE_LENGTH, 0).x)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.ROBOT_LENGTH / 2, 0), 0), Math.toRadians(0))
                .build();
        public Action LEFT_BACKDROP_TO_STACK = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.TILE_LENGTH, 0).x, Positions.GENERAL.STACK_ONE.y, 0), Math.toRadians(180))
                .lineToX(Positions.modifyPose(Positions.GENERAL.STACK_ONE, Constants.ROBOT_LENGTH / 2, 0).x)
                .build();

        public Action STACK_TO_LEFT_BACKDROP = drive.actionBuilder(drive.pose)
                .lineToX(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.TILE_LENGTH, 0).x)
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.ROBOT_LENGTH / 2, 0), 0), Math.toRadians(0))
                .build();

    }

    public class Close {
        // SPIKEMARK
        public Action MID_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(Positions.modifyPose(Positions.CLOSE.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2).y)
                .lineToY(Positions.modifyPose(Positions.CLOSE.SPIKEMARK_MID, 0, Constants.ROBOT_LENGTH / 2).y + 5)
                .setTangent(0)
                .splineToLinearHeading(
                        Positions.modifyPose(
                                Positions.vectorToPose(Positions.GENERAL.BACKDROP_MID, Math.toRadians(0)),
                                -Constants.ROBOT_LENGTH / 2, 0),
                        Math.toRadians(0))
                .build();

        public Action RIGHT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(50)
                .splineTo(Positions.modifyPose(Positions.CLOSE.SPIKEMARK_RIGHT, 5, 5), Math.toRadians(220))
                .setReversed(true)
                .splineTo(Positions.CLOSE.SPIKEMARK_SETUP, Math.toRadians(90))
                .setTangent(0)
                .splineTo(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.ROBOT_LENGTH / 2, 0), Math.toRadians(0))
                .build();

        public Action LEFT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .lineToY(50)
                .splineTo(
                        Positions.modifyPose(Positions.CLOSE.SPIKEMARK_LEFT, -5, 5),
                        Math.toRadians(320))
                .setReversed(true)
                .splineToLinearHeading(
                        Positions.vectorToPose(Positions.CLOSE.SPIKEMARK_SETUP, Math.toRadians(0)),
                        Math.toRadians(90))
                .setTangent(0)
                .splineTo(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.ROBOT_LENGTH / 2, 0), Math.toRadians(0))
                .build();

        public Action PARK = drive.actionBuilder(drive.pose)
                .build();
    }
}
