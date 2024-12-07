package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public final class RedBasket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-32, -60, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(-57.4, -55), Math.toRadians(225))
                        .build());
    }
}

/*
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

 */