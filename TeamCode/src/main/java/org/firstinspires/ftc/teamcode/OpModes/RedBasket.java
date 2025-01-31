package org.firstinspires.ftc.teamcode.OpModes;

import android.hardware.Sensor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.SensorBNO055IMU;
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

        Lift liftSystem = new Lift(hardwareMap);

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

        Actions.runBlocking(drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(-9.75, -28), Math.toRadians(90))
                .waitSeconds(1.5)
    //                                .strafeTo(new Vector2d(-9.75, -35))
                .strafeTo(new Vector2d(-20, -45))
                .strafeTo(new Vector2d(-47, -35))
    //                        .strafeToSplineHeading(new Vector2d(-57.4, -55), Math.toRadians(225))
    //                        .strafeToSplineHeading(new Vector2d(-55, -35), Math.toRadians(90))
                .build()
//                    new Lift.GrabSample(Lift.SAMPLE_POSITION.GROUND),
//                    new SleepAction(0.5),
//                    liftSystem.closeClaw(),
//                    new' SleepAction(0.5),
//                    new Lift.ResetPivot(),)
        );

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

    }
}
