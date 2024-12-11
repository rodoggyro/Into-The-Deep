package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous
public final class RedBasket extends LinearOpMode {
    DistanceSensor leftSensor;

    public class Lift {
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
            Top Basket:
            Bottom Basket:
            Top Chamber:
            Bottom Chamber:
            Bottom of the lift: 0
             */

            public LiftSetPos(int target) {
                this.target = target;
            }

            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    lift.setTargetPosition(target);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                }

                return true;
            }
        }
    }

    public class Pivot {
        private Servo pivot;

        public Pivot (HardwareMap hardwareMap) {
            pivot = hardwareMap.get(Servo.class, "pivotServo");
        }

        /*
            Up: 0.45
            Chamber: 0.285
            Down: 0.08
             */

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
    }

    public class Claw {
        private Servo claw;

        public Claw (HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class clawOpen implements Action {
            private boolean initialized = false;

            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    claw.setPosition(0.5);
                }

                return true;
            }
        }

        public class clawClose implements Action {
            private boolean initialized = false;

            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    claw.setPosition(0);
                }

                return true;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftSensor = hardwareMap.get(DistanceSensor.class, "left");

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashbordTelemetry = dashboard.getTelemetry();
//        dashbordTelemetry.addData("Starting Pos X", -32);
//        dashbordTelemetry.addData("Starting Pos Y", -72 + leftSensor.getDistance(DistanceUnit.INCH) + 9);
//        dashbordTelemetry.update();

//        Pose2d beginPose = new Pose2d(-34, -62, Math.toRadians(90));
        Pose2d beginPose = new Pose2d(-(72 - leftSensor.getDistance(DistanceUnit.INCH) - 9), -62, Math.toRadians(90));
        Pose2d currentPose = beginPose;
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

        Lift lift = new Lift(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        //claw close

        waitForStart();

//        Actions.runBlocking(
//                new SequentialAction(
//                        //lift up to top basket
//                        lift.new LiftSetPos(0),
//                        drive.actionBuilder(beginPose).splineTo(new Vector2d(-57.4, -55), Math.toRadians(225)).build(),
//                        claw.new clawOpen(),
//                        drive.actionBuilder(new Pose2d(-57.4, -55, Math.toRadians(225))).turn(Math.atan((-57.4+47.7)/(-55+40))-Math.toRadians(15)).build(),
//                        //lift down
//                        lift.new LiftSetPos(0)
//                )
//        );

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //lift up to top basket
                        .splineTo(new Vector2d(-57.4, -55), Math.toRadians(225))
                        //claw open
                        .turn(Math.atan((-57.4+47.7)/(-55+40))-Math.toRadians(15))
                        //lift down
                        .lineToYLinearHeading(-40, Math.toRadians(90))
                        //pivot down
                        .waitSeconds(1.5)
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
                        .build());
    }
}
