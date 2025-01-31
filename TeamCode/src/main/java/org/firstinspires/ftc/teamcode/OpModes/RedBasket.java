package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous (name = "Red Bucket Testing")
public final class RedBasket extends LinearOpMode {
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

        Pose2d beginPose = new Pose2d(-9.75, -62, Math.toRadians(90));
        Pose2d currentPose = beginPose;
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

        claw = hardwareMap.get(Servo.class, "claw");
//        lift = new Motor(hardwareMap, "arm", 28*20, (double) 6000 /20);
        lift = hardwareMap.get(DcMotor.class, "arm");
        pivot = hardwareMap.get(Servo.class, "pivotServo");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        lift.resetEncoder();
//        lift.setRunMode(Motor.RunMode.PositionControl);
//        lift.setPositionCoefficient(0.5);
//        lift.setPositionTolerance(15);

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        //TODO: add specimen placing
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(-9.75, -28), Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(-20, -45))
                .strafeTo(new Vector2d(-47, -35))
    //                        .strafeToSplineHeading(new Vector2d(-57.4, -55), Math.toRadians(225))
    //                        .strafeToSplineHeading(new Vector2d(-55, -35), Math.toRadians(90))
                .build()
        );

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (timer.time() < 1000) {
            double target = (0.1666 * Math.log(timer.time()) + 1);
            pivot.setPosition(target);
        }

        pivot.setPosition(0.5);
        sleep(750);
        claw.setPosition(0.3);
        sleep(500);
        pivot.setPosition(0.027);

        lift.setTargetPosition(-2750);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
//            lift.set(0.75);
            lift.setPower(-0.75);
            telemetry.addData("lift pos", lift.getCurrentPosition());
            telemetry.update();
        }

        Actions.runBlocking(drive.actionBuilder(new Pose2d(-46, -35, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-57.4, -55), Math.toRadians(225))
                .build()
        );

        claw.setPosition(0);

        //TODO: go to spike mark 2
        //TODO: pick up sample
        //TODO: drop sample in low bucket
        //TODO: park
    }
}
