package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity LeftPark = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62.58, Math.toRadians(90.00)))
                                .splineToConstantHeading(new Vector2d(-12.13, -60.67), Math.toRadians(90.00))
                                .lineToLinearHeading(new Pose2d(-12.13, -9.79, Math.toRadians(137.73)))
                                .lineToLinearHeading(new Pose2d(-18.04, -4.79, Math.toRadians(137.73)))

                                // TODO: Code to place the preloaded cone on the high goal

                                .lineToLinearHeading(new Pose2d(-12.13, -12.61, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-62, -12.61)) // Go to pick up gone

                                // TODO: Code to pick up the FIRST cone

                                .lineToConstantHeading(new Vector2d(-35, -12.61))
                                .turn(Math.toRadians(47.5)) // turn to 47.5 degrees
                                .lineToConstantHeading(new Vector2d(-28.42, -4.61))

                                // TODO: Code to place the FIRST cone

                                .lineToLinearHeading(new Pose2d(-35, -12.61, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-62, -12.61))

                                // TODO: Code to pick up the SECOND cone

                                .lineToConstantHeading(new Vector2d(-35, -12.61))
                                .turn(Math.toRadians(47.5)) // turn to 47.5 degrees
                                .lineToConstantHeading(new Vector2d(-28.42, -4.61))

                                // TODO: Code to place the SECOND cone

                                .lineToLinearHeading(new Pose2d(-35, -12.61, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-62, -12.61))

                                // TODO: Code to pick up the THIRD cone

                                .lineToConstantHeading(new Vector2d(-35, -12.61))
                                .turn(Math.toRadians(47.5)) // turn to 47.5 degrees
                                .lineToConstantHeading(new Vector2d(-28.42, -4.61))

                                // TODO: Code to place the THIRD cone
                                // And now we go park!

                                .lineToLinearHeading(new Pose2d(-35, -12.61, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-60, -12.61, Math.toRadians(225)))
                                .lineToLinearHeading(new Pose2d(-58, -36, Math.toRadians(90))) // Parking

                                .build()
                );

        RoadRunnerBotEntity MiddlePark = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62.58, Math.toRadians(90.00)))
                                .splineToConstantHeading(new Vector2d(-12.13, -60.67), Math.toRadians(90.00))
                                .lineToLinearHeading(new Pose2d(-12.13, -9.79, Math.toRadians(137.73)))
                                .lineToLinearHeading(new Pose2d(-18.04, -4.79, Math.toRadians(137.73)))

                                // TODO: Code to place the preloaded cone on the high goal

                                .lineToLinearHeading(new Pose2d(-12.13, -12.61, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-62, -12.61)) // Go to pick up gone

                                // TODO: Code to pick up the FIRST cone

                                .lineToConstantHeading(new Vector2d(-35, -12.61))
                                .turn(Math.toRadians(47.5)) // turn to 47.5 degrees
                                .lineToConstantHeading(new Vector2d(-28.42, -4.61))

                                // TODO: Code to place the FIRST cone

                                .lineToLinearHeading(new Pose2d(-35, -12.61, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-62, -12.61))

                                // TODO: Code to pick up the SECOND cone

                                .lineToConstantHeading(new Vector2d(-35, -12.61))
                                .turn(Math.toRadians(47.5)) // turn to 47.5 degrees
                                .lineToConstantHeading(new Vector2d(-28.42, -4.61))

                                // TODO: Code to place the SECOND cone

                                .lineToLinearHeading(new Pose2d(-35, -12.61, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-62, -12.61))

                                // TODO: Code to pick up the THIRD cone

                                .lineToConstantHeading(new Vector2d(-35, -12.61))
                                .turn(Math.toRadians(47.5)) // turn to 47.5 degrees
                                .lineToConstantHeading(new Vector2d(-28.42, -4.61))

                                // TODO: Code to place the THIRD cone
                                // And now we go park!

                                .lineToLinearHeading(new Pose2d(-35, -12.61, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-36, -12.61, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(-90))) // Parking

                                .build()
                );

        RoadRunnerBotEntity RightPark = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62.58, Math.toRadians(90.00)))
                                .splineToConstantHeading(new Vector2d(-12.13, -60.67), Math.toRadians(90.00))
                                .lineToLinearHeading(new Pose2d(-12.13, -9.79, Math.toRadians(137.73)))
                                .lineToLinearHeading(new Pose2d(-18.04, -4.79, Math.toRadians(137.73)))

                                // TODO: Code to place the preloaded cone on the high goal

                                .lineToLinearHeading(new Pose2d(-12.13, -12.61, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-62, -12.61)) // Go to pick up gone

                                // TODO: Code to pick up the FIRST cone

                                .lineToConstantHeading(new Vector2d(-35, -12.61))
                                .turn(Math.toRadians(47.5)) // turn to 47.5 degrees
                                .lineToConstantHeading(new Vector2d(-28.42, -4.61))

                                // TODO: Code to place the FIRST cone

                                .lineToLinearHeading(new Pose2d(-35, -12.61, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-62, -12.61))

                                // TODO: Code to pick up the SECOND cone

                                .lineToConstantHeading(new Vector2d(-35, -12.61))
                                .turn(Math.toRadians(47.5)) // turn to 47.5 degrees
                                .lineToConstantHeading(new Vector2d(-28.42, -4.61))

                                // TODO: Code to place the SECOND cone

                                .lineToLinearHeading(new Pose2d(-35, -12.61, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-62, -12.61))

                                // TODO: Code to pick up the THIRD cone

                                .lineToConstantHeading(new Vector2d(-35, -12.61))
                                .turn(Math.toRadians(47.5)) // turn to 47.5 degrees
                                .lineToConstantHeading(new Vector2d(-28.42, -4.61))

                                // TODO: Code to place the THIRD cone
                                // And now we go park!

                                .lineToLinearHeading(new Pose2d(-35, -12.61, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-12, -12.61, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(-90))) // Parking

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(LeftPark)
                .addEntity(MiddlePark)
                .addEntity(RightPark)
                .start();
    }
}