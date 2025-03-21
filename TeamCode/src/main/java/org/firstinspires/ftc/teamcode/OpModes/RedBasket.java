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

@Autonomous (name = "Bucket w/ Specimen (Dual Color)")

public final class RedBasket extends LinearOpMode {
    BNO055IMU imu;

    Servo claw;
    DcMotor lift;
    Servo pivot;

    int liftPosition = -625;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        Pose2d beginPose = new Pose2d(-9.5, -60, Math.toRadians(90));
        Pose2d currentPose = beginPose;
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        claw = hardwareMap.get(Servo.class, "claw");
//        lift = new Motor(hardwareMap, "arm", 28*20, (double) 6000 /20);
        lift = hardwareMap.get(DcMotor.class, "arm");
        pivot = hardwareMap.get(Servo.class, "pivotServo");
        pivot.setPosition(0.027);
        claw.setPosition(0.5);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        lift.resetEncoder();
//        lift.setRunMode(Motor.RunMode.PositionControl);
//        lift.setPositionCoefficient(0.5);
//        lift.setPositionTolerance(15);

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        lift.setTargetPosition(liftPosition);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-0.75);
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(-8, -35), Math.toRadians(90))
                .build());

        pivot.setPosition(0.5);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(-8, -35, Math.toRadians(90)))
                .strafeTo(new Vector2d(-8, -40))
                .build());

        claw.setPosition(0);

        pivot.setPosition(0.027);

        lift.setTargetPosition(-50);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        sleep(500);

        //TODO: add specimen placing
        Actions.runBlocking(drive.actionBuilder(new Pose2d(new Vector2d(-8, -40), Math.toRadians(90)))
                .strafeTo(new Vector2d(-24, -41))
                .build()
        );

        pivot.setPosition(0.35);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(-8, -40, Math.toRadians(90)))
                .strafeTo(new Vector2d(-45, -42))
                .build());

        pivot.setPosition(0.57);

        sleep(750);

        claw.setPosition(0.5);

        sleep(500);

        pivot.setPosition(0.027);

        lift.setTargetPosition(-850);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-1);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(new Vector2d(-47, -42), Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-56, -52), Math.toRadians(225))
                .build()
        );

        claw.setPosition(0);

        sleep(250);

        lift.setTargetPosition(-50);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
//
        Actions.runBlocking(drive.actionBuilder(new Pose2d(-56, -52, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(-57, -43), Math.toRadians(90))
                .build()
        );

        while (lift.isBusy()) {
            sleep(10);
        }

        pivot.setPosition(0.45);
        sleep(500);

        pivot.setPosition(0.57);
        sleep(750);
        claw.setPosition(0.5);
        sleep(750);
        pivot.setPosition(0.027);

        lift.setTargetPosition(-850);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
            lift.setPower(-1);
            sleep(10);
        }

        Actions.runBlocking(drive.actionBuilder(new Pose2d(new Vector2d(-57, -42), Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-56, -52), Math.toRadians(225))
                .build()
        );

        claw.setPosition(0);
        sleep(500);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(new Vector2d(-56, -52), Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(-48, 0), Math.toRadians(0))
                .strafeTo(new Vector2d(-0, 0))
                .build()
        );

        pivot.setPosition(0.25);
        sleep(500);
    }
}
