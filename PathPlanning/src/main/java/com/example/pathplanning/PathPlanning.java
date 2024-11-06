package com.example.pathplanning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.ColorScheme;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PathPlanning {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Vector2d[] redBucketSpikeMarks = {
            new Vector2d(-47.7, -40),
            new Vector2d(-57.4, -40),
            new Vector2d(-55, -24)
        };

        Vector2d[] blueBucketSpikeMarks = {
            new Vector2d(47.7, 40),
            new Vector2d(57.4, 40),
            new Vector2d(55, 24)
        };

        RoadRunnerBotEntity redBucket = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-32, -60, Math.toRadians(90)))
                        .addDisplacementMarker(1.5, () -> {
                            System.out.println(Math.toDegrees(Math.sin(9.4/31.2)));
                        })
                        .splineTo(new Vector2d(-57.4, -55), Math.toRadians(225))
                        .waitSeconds(1.5)
                        .lineToSplineHeading(new Pose2d(redBucketSpikeMarks[0], Math.toRadians(90)))
                        .forward(5)
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(-57.4, -55, Math.toRadians(225)))
                        .waitSeconds(1.5)
                        .forward(-10)
                        .lineToSplineHeading(new Pose2d(redBucketSpikeMarks[1], Math.toRadians(90)))
                        .forward(5)
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(-57.4, -55, Math.toRadians(225)))
                        .waitSeconds(1.5)
                        .forward(-10)
                        .lineToSplineHeading(new Pose2d(redBucketSpikeMarks[2], Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(-57.4, -55, Math.toRadians(225)))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(-26, 0, Math.toRadians(0)))
                        .build());

        RoadRunnerBotEntity redObservation = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(20, -60, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(8, -33), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .lineTo(new Vector2d(48, -40))
                        .forward(5)
                        .lineToSplineHeading(new Pose2d(55, -55, Math.toRadians(-90)))
                        .lineToConstantHeading(new Vector2d(50, -45))
                        .waitSeconds(1.5)
                        .forward(10)
                        .lineToSplineHeading(new Pose2d(9, -33, Math.toRadians(90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(50, -55, Math.toRadians(-90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(9, -33, Math.toRadians(90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(50, -55, Math.toRadians(-90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(9, -33, Math.toRadians(90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(50, -55, Math.toRadians(-90)))
                        .build());

        RoadRunnerBotEntity blueBucket = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(32, 60, Math.toRadians(-90)))
                        .addDisplacementMarker(1.5, () -> {
                            System.out.println(Math.toDegrees(Math.sin(9.4/31.2)));
                        })
                        .splineTo(new Vector2d(57.4, 55), Math.toRadians(45))
                        .waitSeconds(1.5)
                        .lineToSplineHeading(new Pose2d(blueBucketSpikeMarks[0], Math.toRadians(-90)))
                        .forward(5)
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(57.4, 55, Math.toRadians(45)))
                        .waitSeconds(1.5)
                        .forward(-10)
                        .lineToSplineHeading(new Pose2d(blueBucketSpikeMarks[1], Math.toRadians(-90)))
                        .forward(5)
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(57.4, 55, Math.toRadians(45)))
                        .waitSeconds(1.5)
                        .forward(-10)
                        .lineToSplineHeading(new Pose2d(blueBucketSpikeMarks[2], Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(57.4, 55, Math.toRadians(45)))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(26, 0, Math.toRadians(180)))
                        .build());

        RoadRunnerBotEntity blueObservation = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-20, 60, Math.toRadians(-90)))
                        .splineToConstantHeading(new Vector2d(-8, 33), Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .lineTo(new Vector2d(-48, 40))
                        .forward(5)
                        .lineToSplineHeading(new Pose2d(-55, 55, Math.toRadians(90)))
                        .lineToConstantHeading(new Vector2d(-50, 45))
                        .waitSeconds(1.5)
                        .forward(10)
                        .lineToSplineHeading(new Pose2d(-9, 33, Math.toRadians(-90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(-50, 55, Math.toRadians(90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(-9, 33, Math.toRadians(-90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(-50, 55, Math.toRadians(90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(-9, 33, Math.toRadians(-90)))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(-50, 55, Math.toRadians(90)))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBucket)
                .setShowFPS(true)
                .addEntity(redObservation)
                .addEntity(blueBucket)
                .addEntity(blueObservation)
                .start();
    }
}