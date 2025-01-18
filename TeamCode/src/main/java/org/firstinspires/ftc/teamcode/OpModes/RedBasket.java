package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.SensorBNO055IMU;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Disabled
@Autonomous (name = "DO NOT USE")
public final class RedBasket extends LinearOpMode {
    DistanceSensor leftSensor;
    BNO055IMU imu;
    private DcMotorEx lift;
    private Servo claw;
    private Servo pivot;

    /*public class Lift {
        private DcMotorEx lift;

        public Lift (HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "arm");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class LiftSetPos implements Action {
            private boolean initialized = false;
            private int target;
            /*
            Top Basket: -5400
            Bottom Basket: -2800
            Top Chamber: -3900
            Bottom Chamber: -2200
            Bottom of the lift: 0


            public LiftSetPos(int target) {
                this.target = target;
            }

            public boolean run(@NonNull TelemetryPacket packet) {
                initialized = true;
                lift.setTargetPosition(target);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);

                return true;
            }
        }
    }
    */

    /*public class Pivot {
        private Servo pivot;

        public Pivot (HardwareMap hardwareMap) {
            pivot = hardwareMap.get(Servo.class, "pivotServo");
        }

        /*
            Up: 0.45
            Chamber: 0.285
            Down: 0.08


        public class PivotPos implements Action {
            private boolean initialized = false;
            private double target;

            public PivotPos(double target) {
                this.target = target;
            }

            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    pivot.setPosition(target);
                }

                return true;
            }
        }

        public class PivotUp implements Action {
            private boolean initialized = false;


            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    pivot.setPosition(0.45);
                }

                return true;
            }
        }

        public class PivotChamber implements Action {
            private boolean initialized = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    pivot.setPosition(0.285);
                }

                return true;
            }
        }

        public class PivotDown implements Action {
            private boolean initialized = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    pivot.setPosition(0.08);
                }

                return true;
            }
        }
    }*/

    /*public class Claw {


        public Claw (HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class ClawOpen implements Action {
            private boolean initialized = false;

            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    claw.setPosition(0);
                }

                return true;
            }
        }

        public class ClawClose implements Action {
            private boolean initialized = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    claw.setPosition(1);
                }

                return true;
            }
        }
    }*/

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        leftSensor = hardwareMap.get(DistanceSensor.class, "left");

        claw = hardwareMap.get(Servo.class, "claw");

        pivot = hardwareMap.get(Servo.class, "pivotServo");

        lift = hardwareMap.get(DcMotorEx.class, "arm");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        imu = hardwareMap.get(BNO055IMU.class, "imu");

//        imu.initialize(parameters);

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("X Position", -(72 - (leftSensor.getDistance(DistanceUnit.INCH) + 9)));
        telemetry.update();

//        Pose2d beginPose = new Pose2d(-34, -62, Math.toRadians(90));
        Pose2d beginPose = new Pose2d(-34, -62, Math.toRadians(90));
        Pose2d currentPose = beginPose;
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

//        Lift lift = new Lift(hardwareMap);
//        Pivot pivot = new Pivot(hardwareMap);
//        Claw claw = new Claw(hardwareMap);
//
//        //claw close
//        Actions.runBlocking(claw. new ClawClose());

        claw.setPosition(1);
        pivot.setPosition(0.027);

        waitForStart();

        lift.setTargetPosition(-5400);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
                                //lift up to top basket
                                .splineTo(new Vector2d(-50, -54.5), Math.toRadians(215))
                                .lineToX(-56)
                                //claw open
                                .build()
                )
        );

        while(lift.isBusy()){
            sleep(5);
        }

        pivot.setPosition(0.285);

        claw.setPosition(0);

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(-56, -54.5, Math.toRadians(215)))
                                .splineTo(new Vector2d(-45, 0), Math.toRadians(180))
                                .lineToX(-26)
                                .build()
                )
        );

//        claw. new ClawOpen();
//                        claw. new ClawOpen(),
//                        drive.actionBuilder(new Pose2d(-57.4, -55, Math.toRadians(225)))
//                                .turn(Math.atan((-57.4+47.7)/(-55+40))-Math.toRadians(15))
//                                //lift down
//                                .build(),
////                        lift. new LiftSetPos(0),
//                        drive.actionBuilder(new Pose2d(-57.4, -55, 225 + (Math.atan((-57.4+47.7)/(-55+40))-Math.toRadians(15))))
//                                .lineToYLinearHeading(-40, Math.toRadians(90))
//                                .build(),
////                        pivot. new PivotDown(),
////                        claw. new ClawClose(),
//                        drive.actionBuilder( new Pose2d(new Vector2d(-47.7, -40), Math.toRadians(90)))
//                                .build()

                        //close claw
                        //pivot up
                        //lift up to top basket
//                        .splineToSplineHeading(new Pose2d(-57.4, -55, Math.toRadians(225)), Math.toRadians(225))
//                        //claw open
//                        //lift down
//                        .splineTo(new Vector2d(-57.4, -40), Math.toRadians(90))
//                        //pivot down
//                        .waitSeconds(1.5)
//                        //close claw
//                        //pivot up
//                        //lift up to top basket
//                        .splineTo(new Vector2d(-57.4, -55), Math.toRadians(225))
//                        //claw open
//                        //lift down
//                        .splineTo(new Vector2d(-55, -24), Math.toRadians(180))
//                        //pivot down
//                        .waitSeconds(1.5)
//                        //close claw
//                        //pivot up
//                        //lift up to top basket
//                        .splineTo(new Vector2d(-57.4, -55), Math.toRadians(225))
//                        //claw open
//                        //lift down
//                        .lineToY(-40)
//                        .splineTo(new Vector2d(-45, 0), Math.toRadians(180))
//                        .lineToX(-26)
    }
}
