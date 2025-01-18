package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;


@TeleOp

public class ArcadeDrive extends LinearOpMode {

    BNO055IMU imu;

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

    boolean endGame = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, -0, Math.toRadians(0)));

        telemetry.addData("linear Scalar", drive.linearScalarWorks);
        telemetry.update();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        if(!imu.isGyroCalibrated()){
            imu.initialize(parameters);
        }

        lift = hardwareMap.dcMotor.get("arm");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotServo = hardwareMap.servo.get("pivotServo");
        pivotServo.setPosition(0.027);

        //Hanging
        hanging = hardwareMap.servo.get("hanging");
        hanging.setPosition(0);
        winch = hardwareMap.dcMotor.get("winch");

        //Servos
        claw = hardwareMap.servo.get("claw");

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        double turning;

        while (opModeIsActive()) {
            turning = -0.75 * gamepad1.right_stick_x;

            if (gamepad1.left_bumper) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                1
                        ),
                        turning
                ));
            } else if (gamepad1.right_bumper) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -1
                        ),
                        turning
                ));
            } else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0
                        ),
                        turning
                ));
            }

            if ((lift.getCurrentPosition() > -2589)){
                if (gamepad2.left_trigger > 0.1){
                    lift.setPower(gamepad2.left_trigger);
                } else if (gamepad2.right_trigger > 0.1){
                    lift.setPower(-gamepad2.right_trigger);
                } else {
                    lift.setPower(0);
                }
            } else if (lift.getCurrentPosition() <= -2589) {
                if (gamepad2.left_trigger > 0.1){
                    lift.setPower(gamepad2.left_trigger);
                } else {
                    lift.setPower(0);
                }
            }

            if (gamepad2.back){
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.b && !previousButtonState && clawDebounceComplete) {
                clawOpen = !clawOpen;
                clawDebounceComplete = false;
                clawDebounceStartTime = System.currentTimeMillis();
            }

            if (!clawDebounceComplete && (System.currentTimeMillis() - clawDebounceStartTime) >= debounceDelay){
                clawDebounceComplete = true;
            }

            previousButtonState = gamepad2.b;

            if (clawOpen) {
                claw.setPosition(1);
            } else {
                claw.setPosition(0);
            }

            if (timer.time() > 80 && !endGame) {
                gamepad1.rumble(0.75, 0.75, 1500);
                gamepad2.rumble(0.75, 0.75, 1500);
                endGame = true;
            }

            if (gamepad1.back){
                endGame = true;
            }

            if (endGame) {
                if (gamepad2.dpad_left) {
                    hanging.setPosition(0.75);
                } else if (gamepad2.dpad_right) {
                    hanging.setPosition(0);
                }

                if (gamepad1.a) {
                    winch.setPower(-1);
                } else if (gamepad2.x) {
                    winch.setPower(1);
                } else {
                    winch.setPower(0);
                }
            }

            if (gamepad2.y && !previousPivotState && pivotDebounceComplete) {
                pivotState ++;
                pivotDebounceComplete = false;
                pivotDebounceStartTime = System.currentTimeMillis();
            }

            if (!pivotDebounceComplete && (System.currentTimeMillis() - pivotDebounceStartTime) >= debounceDelay){
                pivotDebounceComplete = true;
            }

            previousButtonState = gamepad2.y;

            switch (pivotState % 3){
                case 0:
                    pivotServo.setPosition(0.027);
                    break;
                case 1:
                    pivotServo.setPosition(0.33);
                    break;
                case 2:
                    pivotServo.setPosition(0.5);
                    break;
            }

//            pivotServo.setPosition(gamepad2.left_stick_y);

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("pivot Position", pivotServo.getPosition());
            telemetry.addData("lift", lift.getCurrentPosition());
            telemetry.addData("hanging", hanging.getPosition());
//            telemetry.addData("Heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
