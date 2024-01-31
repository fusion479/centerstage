package com.example.meepmeeptesting.main;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, 5, toRadians(60), 15.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63.25, Math.toRadians(90)))
                                .forward(14)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-40, -34, Math.toRadians(140)), Math.toRadians(140))
                                // END OF SPIKE MARK
                                .setTangent(Math.toRadians(320))
                                .splineToLinearHeading(new Pose2d(-34, -38, Math.toRadians(90)), Math.toRadians(270))
                                .lineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(90)))
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(48, -30, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}