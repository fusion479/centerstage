package com.example.meepmeeptesting.main;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, 5, toRadians(180), 15.5)
                .followTrajectorySequence(drive ->
<<<<<<< Updated upstream
                        drive.trajectorySequenceBuilder(new Pose2d(15.5, 63.25, Math.toRadians(-90)))
                                .setReversed(true) //spike 1
                                .splineToLinearHeading(new Pose2d(8, 37, Math.toRadians(-90)), Math.toRadians(135))
=======
                        drive.trajectorySequenceBuilder(new Pose2d(15.5, -63.25, toRadians(-90)))

                                .setReversed(true) //spike 1
                                .splineToLinearHeading(new Pose2d(8, -37, toRadians(-45)), toRadians(135))
                                .lineToLinearHeading(new Pose2d(52, -30.5, toRadians(180)))
                                .setReversed(false)
                                .splineTo(new Vector2d(-10,-12), toRadians(180))
                                .splineTo(new Vector2d(-57,-12), toRadians(180))
                                .setReversed(true) //spike 1
                                .splineTo(new Vector2d(-10,-12), toRadians(0))
                                .splineTo(new Vector2d(51, -28.5), toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(-10,-12), toRadians(180))
                                .splineTo(new Vector2d(-57,-12), toRadians(180))
                                .setReversed(true) //spike 1
                                .splineTo(new Vector2d(-10,-12), toRadians(0))
                                .splineTo(new Vector2d(51, -28.5), toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(-10,-12), toRadians(180))
                                .splineTo(new Vector2d(-57,-20), toRadians(190))
                                .setReversed(true) //spike 1
                                .splineTo(new Vector2d(-10,-12), toRadians(0))
                                .splineTo(new Vector2d(51, -28.5), toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(-10,-12), toRadians(180))
                                .splineTo(new Vector2d(-57,-20), toRadians(190))
                                .setReversed(true) //spike 1
                                .splineTo(new Vector2d(-10,-12), toRadians(0))
                                .splineTo(new Vector2d(51, -28.5), toRadians(0))

>>>>>>> Stashed changes

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}