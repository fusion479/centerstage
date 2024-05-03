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
        return this.color == Camera.Color.RED ? new Pose2d(pose.position.x, -pose.position.y, Math.toRadians(360) - pose.heading.real) : pose;
    }

    public class Far {
        // SPIKEMARK
        public Action MID_SPIKEMARK = drive.actionBuilder(drive.pose)
                .build();

        public Action RIGHT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .build();

        public Action LEFT_SPIKEMARK = drive.actionBuilder(drive.pose)
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

    public class Close {
        // SPIKEMARK
        public Action MID_SPIKEMARK = drive.actionBuilder(drive.pose)
                .build();

        public Action RIGHT_SPIKEMARK = drive.actionBuilder(drive.pose)
                .build();

        public Action LEFT_SPIKEMARK = drive.actionBuilder(drive.pose)
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
