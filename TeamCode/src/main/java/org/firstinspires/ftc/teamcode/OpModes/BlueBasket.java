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

@Autonomous (name = "Blue Bucket w/ Specimen")
@Disabled
public final class BlueBasket extends LinearOpMode {
    BNO055IMU imu;

    Servo claw;
    DcMotor lift;
    Servo pivot;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        Pose2d beginPose = new Pose2d(9.75, 62, Math.toRadians(-90));
        Pose2d currentPose = beginPose;
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

        claw = hardwareMap.get(Servo.class, "claw");
//        lift = new Motor(hardwareMap, "arm", 28*20, (double) 6000 /20);
        lift = hardwareMap.get(DcMotor.class, "arm");
        pivot = hardwareMap.get(Servo.class, "pivotServo");
        pivot.setPosition(0.027);
        claw.setPosition(0.3);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        lift.resetEncoder();
//        lift.setRunMode(Motor.RunMode.PositionControl);
//        lift.setPositionCoefficient(0.5);
//        lift.setPositionTolerance(15);

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        lift.setTargetPosition(-575);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-0.75);

        Actions.runBlocking(drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(9.75, 38), Math.toRadians(-90))
                .build()
        );

        while (lift.isBusy()) {
            sleep(10);
        }
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(0);

        pivot.setPosition(0.15);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(9.75, 32, Math.toRadians(-90)))
                .strafeTo(new Vector2d(9.75, 40))
                .build()
        );

        claw.setPosition(0);

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        sleep(500);

        //TODO: add specimen placing
        Actions.runBlocking(drive.actionBuilder(new Pose2d(new Vector2d(9.75, 28), Math.toRadians(-90)))
                .strafeTo(new Vector2d(20, 45))
                .strafeTo(new Vector2d(46, 38))
                .build()
        );

        while (lift.isBusy()) {
            sleep(10);
        }
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(0);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        pivot.setPosition(0.25);
        sleep(500);

        pivot.setPosition(0.53);
        sleep(750);
        claw.setPosition(0.3);
        sleep(500);
        pivot.setPosition(0.027);

        lift.setTargetPosition(-2750 / 5);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (lift.isBusy()) {
//            lift.set(0.75);
            lift.setPower(-1);
            telemetry.addData("lift pos", lift.getCurrentPosition());
            telemetry.update();
        }

        Actions.runBlocking(drive.actionBuilder(new Pose2d(new Vector2d(46, 38), Math.toRadians(-90)))
                .strafeToSplineHeading(new Vector2d(57, 55), Math.toRadians(45))
                .build()
        );

        claw.setPosition(0);

        sleep(250);

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(54, 53, Math.toRadians(45)))
                .strafeToSplineHeading(new Vector2d(56, 38), Math.toRadians(-90))
                .build()
        );

        while (lift.isBusy()) {
            sleep(10);
        }

        pivot.setPosition(0.45);
        sleep(500);

        pivot.setPosition(0.53);
        sleep(750);
        claw.setPosition(0.3);
        sleep(500);
        pivot.setPosition(0.027);

        lift.setTargetPosition(-2750 / 5);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
            lift.setPower(-1);
            sleep(10);
        }

        Actions.runBlocking(drive.actionBuilder(new Pose2d(new Vector2d(46, 38), Math.toRadians(-90)))
                .strafeToSplineHeading(new Vector2d(57, 55), Math.toRadians(45))
                .build()
        );

        claw.setPosition(0);
        sleep(500);

        Actions.runBlocking(drive.actionBuilder(new Pose2d(new Vector2d(52, 50), Math.toRadians(45)))
                .strafeToSplineHeading(new Vector2d(48, 0), Math.toRadians(180))
                .strafeTo(new Vector2d(0, 0))
                .build()
        );

        pivot.setPosition(0.25);
        sleep(500);
    }
}
