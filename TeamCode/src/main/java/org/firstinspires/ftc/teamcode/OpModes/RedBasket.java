package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous
public final class RedBasket extends LinearOpMode {
    DistanceSensor leftSensor;


    @Override
    public void runOpMode() throws InterruptedException {
//        leftSensor = hardwareMap.get(DistanceSensor.class, "left");

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashbordTelemetry = dashboard.getTelemetry();
//        dashbordTelemetry.addData("Starting Pos X", -32);
//        dashbordTelemetry.addData("Starting Pos Y", -72 + leftSensor.getDistance(DistanceUnit.INCH) + 9);
//        dashbordTelemetry.update();

        Pose2d beginPose = new Pose2d(-34, -62, Math.toRadians(90));
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


 public class Lift {
        private DcMotorEx lift;

        public Lift (HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "arm");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class LiftToTop implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-1);
                    initialized = true;
                }

                return true;
            }
        }
    }

 */