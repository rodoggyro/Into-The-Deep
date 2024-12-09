package com.example.pathplanning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-32, -62, Math.toRadians(90)))
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

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}