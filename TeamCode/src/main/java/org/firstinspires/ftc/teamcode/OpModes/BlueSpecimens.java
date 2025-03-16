package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous (name = "Blue Observation Zone")

public final class BlueSpecimens extends LinearOpMode {

    double wallPosition = -48;
    int liftPosition = -625;
    int chamberPosition = -32;

    double pivotPickup = 0.30;

    double XoffsetInches = 7;

    boolean continuePath = false;

    Servo claw;
    DcMotor lift;
    Servo pivot;

    DistanceSensor rightSensor;
    DistanceSensor forwardSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
        forwardSensor = hardwareMap.get(DistanceSensor.class, "forwardSensor");

//        Pose2d beginPose = new Pose2d(72 - rightSensor.getDistance(DistanceUnit.INCH), -60, Math.toRadians(90));
        Pose2d beginPose = new Pose2d(9.5, -60, Math.toRadians(90));
        Pose2d currentPose = beginPose;
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(DcMotor.class, "arm");
        pivot = hardwareMap.get(Servo.class, "pivotServo");
        claw.setPosition(0.55);
        sleep(250);
        pivot.setPosition(0.027);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        lift.setTargetPosition(liftPosition);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-0.75);
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(8, -35), Math.toRadians(90))
                .build());

        pivot.setPosition(0.5);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(7, -35, Math.toRadians(90)))
                .strafeTo(new Vector2d(7, -40))
                .build());

        claw.setPosition(0);
        pivot.setPosition(pivotPickup);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(7, -40, Math.toRadians(90)))
                .strafeTo(new Vector2d(36, -37))
                .build());

        pivot.setPosition(0.027);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.75);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(36, -37, Math.toRadians(90)))
                .strafeTo(new Vector2d(36, -37))
                .strafeTo(new Vector2d(36, -10))
                .strafeTo(new Vector2d(47.2, -15))
                .strafeTo(new Vector2d(47.2, -55))
                .strafeTo(new Vector2d(47.2, -10))
                .strafeTo(new Vector2d(57.5, -15))
                .strafeTo(new Vector2d(57.5, -55))
                .strafeToSplineHeading(new Vector2d(57.5, -47), Math.toRadians(270))
                .build());

        pivot.setPosition(pivotPickup);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(57.5, -47, Math.toRadians(270)))
                .strafeTo(new Vector2d(57.5, wallPosition))
                .build());

        claw.setPosition(0.5);

        sleep(500);

        lift.setTargetPosition(liftPosition-50);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-0.75);
        pivot.setPosition(0.075);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(57.5, wallPosition, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(10, -33), Math.toRadians(90))
                .build());

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.75);

        sleep(250);

        pivot.setPosition(0.075);
        claw.setPosition(0);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(10, -33, Math.toRadians(90)))
                .strafeTo(new Vector2d(10, -40))
                .build());

        pivot.setPosition(pivotPickup);

        /*

         */

        Actions.runBlocking(drive.actionBuilder(new Pose2d(10, -40, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(57.75, wallPosition+10), Math.toRadians(270))
                .strafeTo(new Vector2d(57.75, wallPosition))
                .build());

        claw.setPosition(0.55);

        sleep (500);

        pivot.setPosition(0.1);

        lift.setTargetPosition(liftPosition-150);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-0.75);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(57.75, wallPosition, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(13, -34), Math.toRadians(90))
                .build());

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.75);

        sleep(750);

        pivot.setPosition(0.075);
        claw.setPosition(0);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(13, -33.5, Math.toRadians(90)))
                .strafeTo(new Vector2d(13, -40))
                .build());

        pivot.setPosition(pivotPickup);
    }
}
