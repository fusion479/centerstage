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

                .build();

        public Action RIGHT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .build();

        public Action LEFT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(Positions.vectorToPose(Positions.GENERAL.STACK_ONE, Math.toRadians(0)), Math.toRadians(180))
                .build();

        // BACKDROP
        public Action MID_BACKDROP = drive.actionBuilder(drive.pose)
                .build();

        public Action RIGHT_BACKDROP = drive.actionBuilder(drive.pose)
                .build();

        public Action LEFT_BACKDROP = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToConstantHeading(Positions.modifyPose(Positions.GENERAL.BACKDROP_LEFT, -Constants.ROBOT_LENGTH / 2, 0), Math.toRadians(90))
                .build();

        // PARK
        public Action PARK = drive.actionBuilder(drive.pose)
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
                .splineToLinearHeading(Positions.vectorToPose(Positions.modifyPose(Positions.GENERAL.BACKDROP_RIGHT, -Constants.ROBOT_LENGTH / 2, 0), 0), Math.toRadians(0)).build();

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

        // BACKDROP
        public Action MID_BACKDROP = drive.actionBuilder(drive.pose)
                .build();

        public Action RIGHT_BACKDROP = drive.actionBuilder(drive.pose)
                .build();

        public Action LEFT_BACKDROP = drive.actionBuilder(drive.pose)
                .build();

        // PARK
        public Action PARK = drive.actionBuilder(drive.pose)
                .build();
    }
}
