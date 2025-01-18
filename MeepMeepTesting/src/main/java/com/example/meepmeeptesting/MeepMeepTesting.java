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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redBucket.runAction(redBucket.getDrive().actionBuilder(new Pose2d(-32, -62, Math.toRadians(90)))
                //claw close
                //lift up to top basket
                .splineTo(new Vector2d(-57.4, -55), Math.toRadians(225))
                //claw open
                .turn(Math.atan((-57.4+47.7)/(-55+40))-Math.toRadians(15))
                //lift down
                .lineToYLinearHeading(-40, Math.toRadians(90))
                //pivot down
                .waitSeconds(1.5)
                //close claw
                //pivot up
                //lift up to top basket
                .splineToSplineHeading(new Pose2d(-57.4, -55, Math.toRadians(225)), Math.toRadians(225))
                //claw open
                //lift down
                .splineTo(new Vector2d(-57.4, -40), Math.toRadians(90))
                //pivot down
                .waitSeconds(1.5)
                //close claw
                //pivot up
                //lift up to top basket
                .splineTo(new Vector2d(-57.4, -55), Math.toRadians(225))
                //claw open
                //lift down
                .splineTo(new Vector2d(-55, -24), Math.toRadians(180))
                //pivot down
                .waitSeconds(1.5)
                //close claw
                //pivot up
                //lift up to top basket
                .splineTo(new Vector2d(-57.4, -55), Math.toRadians(225))
                //claw open
                //lift down
                .lineToY(-40)
                .splineTo(new Vector2d(-45, 0), Math.toRadians(180))
                .lineToX(-26)
                .build());

        RoadRunnerBotEntity redObservation = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redObservation.runAction(redObservation.getDrive().actionBuilder(new Pose2d(20, -60, Math.toRadians(90)))
                //claw close
                //lift up to low chamber
                .splineToConstantHeading(new Vector2d(8, -33), Math.toRadians(90))
                //pivot down
                .lineToY(-40)
                //claw open
                //lift down to ground
                .strafeTo(new Vector2d(48, -40))
                //claw close
                //pivot up half
                .splineTo(new Vector2d(55, -55), Math.toRadians(180))
                //claw open
                .strafeTo(new Vector2d(55, -45))
                //pivot down
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(55, -55))
                //claw close
                //pivot up
                .strafeTo(new Vector2d(9, -37))
                //lift up
                .turnTo(Math.toRadians(90))
                //pivot down
                .strafeTo(new Vector2d(9, -33))
                .strafeTo(new Vector2d(9, -40))
                //claw open
                //lift down
                //pivot half
                .strafeToSplineHeading(new Vector2d(55, -45), Math.toRadians(270))
                //pivot down
                .strafeTo(new Vector2d(55, -55))
                //claw close
                //pivot up
                .strafeTo(new Vector2d(10, -37))
                //lift up
                .turnTo(Math.toRadians(90))
                //pivot down
                .strafeTo(new Vector2d(10, -33))
                .strafeTo(new Vector2d(10, -40))
                .strafeTo(new Vector2d(55, -55))
                .build());

        RoadRunnerBotEntity blueBucket = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        blueBucket.runAction(blueBucket.getDrive().actionBuilder(new Pose2d(32, 62, Math.toRadians(-90)))
                //claw close
                //lift up to top basket
                .splineTo(new Vector2d(57.4, 55), Math.toRadians(225-180))
                //claw open
                .turn((Math.atan((-57.4+47.7)/(-55+40))-Math.toRadians(15))-Math.toRadians(180))
                //lift down
                .lineToYLinearHeading(40, Math.toRadians(-90))
                //pivot down
                .waitSeconds(1.5)
                //close claw
                //pivot up
                //lift up to top basket
                .splineToSplineHeading(new Pose2d(57.4, 55, Math.toRadians(225-180)), Math.toRadians(225-180))
                //claw open
                //lift down
                .splineTo(new Vector2d(57.4, 40), Math.toRadians(-90))
                //pivot down
                .waitSeconds(1.5)
                //close claw
                //pivot up
                //lift up to top basket
                .splineTo(new Vector2d(57.4, 55), Math.toRadians(225-180))
                //claw open
                //lift down
                .splineTo(new Vector2d(55, 24), Math.toRadians(0))
                //pivot down
                .waitSeconds(1.5)
                //close claw
                //pivot up
                //lift up to top basket
                .splineTo(new Vector2d(57.4, 55), Math.toRadians(225-180))
                //claw open
                //lift down
                .lineToY(40)
                .splineTo(new Vector2d(45, 0), Math.toRadians(0))
                .lineToX(26)
                .build());

        RoadRunnerBotEntity blueObservation = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        blueObservation.runAction(blueObservation.getDrive().actionBuilder(new Pose2d(-20, 60, Math.toRadians(-90)))
                //claw close
                //lift up to low chamber
                .splineToConstantHeading(new Vector2d(-8, 33), Math.toRadians(-90))
                //pivot down
                .lineToY(40)
                //claw open
                //lift down to ground
                .strafeTo(new Vector2d(-48, 40))
                //claw close
                //pivot up half
                .splineTo(new Vector2d(-55, 55), Math.toRadians(0))
                //claw open
                .strafeTo(new Vector2d(-55, 45))
                //pivot down
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(-55, 55))
                //claw close
                //pivot up
                .strafeTo(new Vector2d(-9, 37))
                //lift up
                .turnTo(Math.toRadians(-90))
                //pivot down
                .strafeTo(new Vector2d(-9, 33))
                .strafeTo(new Vector2d(-9, 40))
                //claw open
                //lift down
                //pivot half
                .strafeToSplineHeading(new Vector2d(-55, 45), Math.toRadians(90))
                //pivot down
                .strafeTo(new Vector2d(-55, 55))
                //claw close
                //pivot up
                .strafeTo(new Vector2d(-10, 37))
                //lift up
                .turnTo(Math.toRadians(-90))
                //pivot down
                .strafeTo(new Vector2d(-10, 33))
                .strafeTo(new Vector2d(-10, 40))
                .strafeTo(new Vector2d(-55, 55))
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