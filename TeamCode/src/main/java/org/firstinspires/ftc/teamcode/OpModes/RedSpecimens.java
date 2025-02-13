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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous (name = "Red Observation Zone")

public final class RedSpecimens extends LinearOpMode {

    int wallPosition = -55;

    Servo claw;
    DcMotor lift;
    Servo pivot;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(9.5, -60, Math.toRadians(90));
        Pose2d currentPose = beginPose;
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(DcMotor.class, "arm");
        pivot = hardwareMap.get(Servo.class, "pivotServo");
        pivot.setPosition(0.027);
        claw.setPosition(0.3);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        lift.setTargetPosition(-575);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-0.75);
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(0, -33), Math.toRadians(90))
                .build());

        pivot.setPosition(0.33);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, -33, Math.toRadians(90)))
                .strafeTo(new Vector2d(0, -40))
                .build());

        claw.setPosition(0);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, -40, Math.toRadians(90)))
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
                .strafeTo(new Vector2d(47.2, wallPosition))
                .strafeTo(new Vector2d(47.2, -10))
                .strafeTo(new Vector2d(57.5, -15))
                .strafeTo(new Vector2d(57.5, wallPosition))
                .strafeToSplineHeading(new Vector2d(57.5, -47), Math.toRadians(270))
                .build());

        pivot.setPosition(0.33);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(57.5, -47, Math.toRadians(270)))
                .strafeTo(new Vector2d(57.5, wallPosition))
                .build());

        claw.setPosition(0.3);
        sleep(150);
        pivot.setPosition(0.027);

        lift.setTargetPosition(-575);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-0.75);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(57.5, wallPosition, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(4, -33), Math.toRadians(90))
                .build());

        pivot.setPosition(0.33);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(4, -33, Math.toRadians(90)))
                .strafeTo(new Vector2d(4, -40))
                .build());

        claw.setPosition(0);
        sleep(150);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.75);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(4, -40, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(57.5, wallPosition), Math.toRadians(270))
                .build());

        claw.setPosition(0.3);
        sleep(150);
        lift.setTargetPosition(-575);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-0.75);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(57.5, wallPosition, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(8, -33), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -40))
                .build());

        claw.setPosition(0);
        sleep(150);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.75);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(8, -40, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(57.5, wallPosition), Math.toRadians(270))
                .build());

        /*Actions.runBlocking(drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(57.5, wallPosition), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(8, -33), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -40))
                .build());

         */
    }
}
