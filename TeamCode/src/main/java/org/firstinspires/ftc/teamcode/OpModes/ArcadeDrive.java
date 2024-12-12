package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;


@TeleOp

public class ArcadeDrive extends LinearOpMode {

    final long debounceDelay = 200;

    DcMotor winch;
    DcMotor lift;

    Servo hanging;
    Servo claw;

    boolean clawOpen = true;
    boolean previousButtonState = false;
    boolean clawDebounceComplete = true;
    long clawDebounceStartTime = 0;

    long pivotWait = 0;
    int pivotState = 0;

    Servo pivotServo;

    boolean pivotDown = false;
    boolean previousPivotState = false;
    boolean pivotDebounceComplete = true;
    long pivotDebounceStartTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, -0, Math.toRadians(0)));

        telemetry.addData("linear Scalar", drive.linearScalarWorks);
        telemetry.update();

        lift = hardwareMap.dcMotor.get("arm");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotServo = hardwareMap.servo.get("pivotServo");
        pivotServo.setPosition(0);

        //Hanging
        hanging = hardwareMap.servo.get("hanging");
        hanging.setPosition(0);
        winch = hardwareMap.dcMotor.get("winch");

        //Servos
        claw = hardwareMap.servo.get("claw");

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            if (gamepad1.left_trigger > 0.1){
                lift.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1){
                lift.setPower(-gamepad1.right_trigger);
            } else {
                lift.setPower(0);
            }

            if (gamepad1.x && !previousButtonState && clawDebounceComplete) {
                clawOpen = !clawOpen;
                clawDebounceComplete = false;
                clawDebounceStartTime = System.currentTimeMillis();
            }

            if (!clawDebounceComplete && (System.currentTimeMillis() - clawDebounceStartTime) >= debounceDelay){
                clawDebounceComplete = true;
            }

            previousButtonState = gamepad1.x;

            if (clawOpen) {
                claw.setPosition(0.5);
            } else {
                claw.setPosition(0);
            }

            if (gamepad1.dpad_left){
                hanging.setPosition(1);
            }

            if (gamepad1.a){
                winch.setPower(1);
            } else {
                winch.setPower(0);
            }

            if (gamepad2.x){
                winch.setPower(-1);
            } else {
                winch.setPower(0);
            }

            if (gamepad1.y && !previousPivotState && pivotDebounceComplete) {
                if (pivotState != 2){
                    pivotState ++;
                } else {
                    pivotState = 0;
                }
                pivotDebounceComplete = false;
                pivotDebounceStartTime = System.currentTimeMillis();
            }

            if (!pivotDebounceComplete && (System.currentTimeMillis() - pivotDebounceStartTime) >= debounceDelay){
                pivotDebounceComplete = true;
            }

            previousButtonState = gamepad1.y;

            if (pivotState == 0) {
                pivotServo.setPosition(0.45);
            } else if (pivotState == 1){
                pivotServo.setPosition(0.285);
            } else {
                pivotServo.setPosition(0.08);
            }

//            pivotServo.setPosition(gamepad2.left_stick_y);

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("pivot Position", pivotServo.getPosition());
            telemetry.addData("lift", lift.getCurrentPosition());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
