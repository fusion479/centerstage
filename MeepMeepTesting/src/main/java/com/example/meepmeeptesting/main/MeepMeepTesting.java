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
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 63.25, Math.toRadians(270)))

                                .forward(26.5)
                                .back(5)
                                // END OF SPIKEMARK
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-48, 24, Math.toRadians(270)), Math.toRadians(270))
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(0)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(40, 12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}