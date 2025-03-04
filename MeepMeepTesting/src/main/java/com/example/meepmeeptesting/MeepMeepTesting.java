package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity redBucket = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redBucket.runAction(redBucket.getDrive().actionBuilder(new Pose2d(-32, -62, Math.toRadians(90)))
                .splineTo(new Vector2d(-57.4, -55), Math.toRadians(225))
                .lineToY(-52.5)
                .splineToSplineHeading(new Pose2d(-47.7, -40, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(-42.5)
                .splineToSplineHeading(new Pose2d(-57.4, -55, Math.toRadians(225)), Math.toRadians(225))
                .lineToY(-52.5)
                .splineToSplineHeading(new Pose2d(-57.7, -40, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(-42.5)
                .splineToSplineHeading(new Pose2d(-57.4, -55, Math.toRadians(225)), Math.toRadians(225))
                .lineToY(-52.5)
                .splineToSplineHeading(new Pose2d(-52.5, -27.5, Math.toRadians(180)), Math.toRadians(180))
                .lineToX(-50)
                .splineToSplineHeading(new Pose2d(-57.4, -55, Math.toRadians(225)), Math.toRadians(225))
                .lineToY(-52.5)
                .splineToSplineHeading(new Pose2d(-40, 0, 0),0)
                .lineToX(-25)
                .build());

        RoadRunnerBotEntity redObservation = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redObservation.runAction(redObservation.getDrive().actionBuilder(new Pose2d(20, -60, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(8, -33), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -40))
                .splineTo(new Vector2d(36, -37), Math.toRadians(90))
                .splineTo(new Vector2d(36, -10), Math.toRadians(90))
                .splineTo(new Vector2d(47.2, -15), Math.toRadians(-90))
                .splineTo(new Vector2d(47.2, -60), Math.toRadians(-90))
                .splineTo(new Vector2d(47.2, -10), Math.toRadians(90))
                .splineTo(new Vector2d(57.5, -15), Math.toRadians(-90))
                .splineTo(new Vector2d(57.5, -60), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(57.5, -47), Math.toRadians(270))
                .strafeTo(new Vector2d(57.5, -60))
                .strafeTo(new Vector2d(57.5, -50))
                .splineToSplineHeading(new Pose2d(8, -33, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -40))
                .splineToSplineHeading(new Pose2d(38, -50, Math.toRadians(270)), Math.toRadians(270))
                .strafeTo(new Vector2d(38, -60))
                .strafeTo(new Vector2d(38, -50))
                .splineToSplineHeading(new Pose2d(8, -40, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -33))
                .strafeTo(new Vector2d(8, -40))
                .splineToSplineHeading(new Pose2d(38, -50, Math.toRadians(270)), Math.toRadians(270))
                .strafeTo(new Vector2d(38, -60))
                .strafeTo(new Vector2d(38, -50))
                .splineToSplineHeading(new Pose2d(8, -40, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -33))
                .strafeTo(new Vector2d(8, -40))
                .build());

        RoadRunnerBotEntity blueBucket = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        blueBucket.runAction(blueBucket.getDrive().actionBuilder(new Pose2d(32, 62, Math.toRadians(-90)))
                .splineTo(new Vector2d(57.4, 55), Math.toRadians(45))
                .lineToY(52.5)
                .splineToSplineHeading(new Pose2d(47.7, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToY(42.5)
                .splineToSplineHeading(new Pose2d(57.4, 55, Math.toRadians(45)), Math.toRadians(45))
                .lineToY(52.5)
                .splineToSplineHeading(new Pose2d(57.7, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToY(42.5)
                .splineToSplineHeading(new Pose2d(57.4, 55, Math.toRadians(45)), Math.toRadians(45))
                .lineToY(52.5)
                .splineToSplineHeading(new Pose2d(52.5, 27.5, Math.toRadians(-180)), Math.toRadians(-180))
                .lineToX(50)
                .splineToSplineHeading(new Pose2d(57.4, 55, Math.toRadians(45)), Math.toRadians(45))
                .lineToY(52.5)
                .splineToSplineHeading(new Pose2d(40, 0.1, 180),180)
//                .lineToX(25)
                .build());

        RoadRunnerBotEntity blueObservation = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        blueObservation.runAction(blueObservation.getDrive().actionBuilder(new Pose2d(-20, 60, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(-8, 33), Math.toRadians(-90))
                .strafeTo(new Vector2d(-8, 40))
                .splineTo(new Vector2d(-36, 37), Math.toRadians(-90))
                .splineTo(new Vector2d(-36, 10), Math.toRadians(-90))
                .splineTo(new Vector2d(-47.2, 15), Math.toRadians(90))
                .splineTo(new Vector2d(-47.2, 60), Math.toRadians(90))
                .splineTo(new Vector2d(-47.2, 10), Math.toRadians(-90))
                .splineTo(new Vector2d(-57.5, 15), Math.toRadians(90))
                .splineTo(new Vector2d(-57.5, 60), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-57.5, 47), Math.toRadians(-270))
                .strafeTo(new Vector2d(-57.5, 60))
                .strafeTo(new Vector2d(-57.5, 50))
                .splineToSplineHeading(new Pose2d(-8, 33, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeTo(new Vector2d(-8, 40))
                .splineToSplineHeading(new Pose2d(-38, 50, Math.toRadians(-270)), Math.toRadians(-270))
                .strafeTo(new Vector2d(-38, 60))
                .strafeTo(new Vector2d(-38, 50))
                .splineToSplineHeading(new Pose2d(-8, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeTo(new Vector2d(-8, 33))
                .strafeTo(new Vector2d(-8, 40))
                .splineToSplineHeading(new Pose2d(-38, 50, Math.toRadians(-270)), Math.toRadians(-270))
                .strafeTo(new Vector2d(-38, 60))
                .strafeTo(new Vector2d(-38, 50))
                .splineToSplineHeading(new Pose2d(-8, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeTo(new Vector2d(-8, 33))
                .strafeTo(new Vector2d(-8, 40))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBucket)
                .addEntity(redObservation)
                .addEntity(blueBucket)
                .addEntity(blueObservation)
                .start();
    }
}