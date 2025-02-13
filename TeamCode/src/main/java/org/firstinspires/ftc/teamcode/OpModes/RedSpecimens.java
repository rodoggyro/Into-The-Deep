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

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous (name = "Red Observation Zone")

public final class RedSpecimens extends LinearOpMode {

//    Servo claw;
//    DcMotor lift;
//    Servo pivot;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(9.5, -60, Math.toRadians(90));
        Pose2d currentPose = beginPose;
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

//        claw = hardwareMap.get(Servo.class, "claw");
//        lift = hardwareMap.get(DcMotor.class, "arm");
//        pivot = hardwareMap.get(Servo.class, "pivotServo");
//        pivot.setPosition(0.027);
//        claw.setPosition(0.3);
//
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(8, -33), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -40))
                .strafeTo(new Vector2d(36, -37))
                .strafeTo(new Vector2d(36, -10))
                .strafeTo(new Vector2d(47.2, -15))
                .strafeTo(new Vector2d(47.2, -60))
                .strafeTo(new Vector2d(47.2, -10))
                .strafeTo(new Vector2d(57.5, -15))
                .strafeTo(new Vector2d(57.5, -60))
                .strafeToSplineHeading(new Vector2d(57.5, -47), Math.toRadians(270))
                .strafeTo(new Vector2d(57.5, -60))
                .strafeToSplineHeading(new Vector2d(8, -33), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -40))
                .strafeToSplineHeading(new Vector2d(57.5, -60), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(8, -33), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -40))
                .strafeToSplineHeading(new Vector2d(57.5, -60), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(8, -33), Math.toRadians(90))
                .strafeTo(new Vector2d(8, -40))
                .build());
    }
}
