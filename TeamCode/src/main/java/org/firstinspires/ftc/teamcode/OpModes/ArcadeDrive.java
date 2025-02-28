package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.PinpointDrive;


@TeleOp

public class ArcadeDrive extends LinearOpMode {

    BNO055IMU imu;

    final long debounceDelay = 200;

    DcMotor winch;
    DcMotor lift;

    Motor arm;

    Servo hanging;
    Servo claw;

    boolean clawOpen = true;
    boolean previousButtonState = false;
    boolean clawDebounceComplete = true;
    long clawDebounceStartTime = 0;

    Servo pivotServo;


    int target = 0;
    boolean liftHoldPos = false;
    int liftState = 0;

    int pivotState = 0;
    boolean previousPivotState = false;
    boolean pivotDebounceComplete = true;
    long pivotDebounceStartTime = 0;

    ButtonReader liftUp;
    ButtonReader liftDown;

    boolean endGame = false;

    double power;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftUp = new ButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_UP);
        liftDown = new ButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_DOWN);

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, -0, Math.toRadians(0)));

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

//        lift = hardwareMap.dcMotor.get("arm");
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = new Motor(hardwareMap, "arm", 28*4, 6000/4);
//        arm.resetEncoder();
        arm.setRunMode(Motor.RunMode.PositionControl);
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm.setInverted(true);
        arm.setPositionTolerance(10);
        arm.setPositionCoefficient(0.2);

        pivotServo = hardwareMap.servo.get("pivotServo");

        //Hanging
        hanging = hardwareMap.servo.get("hanging");

        winch = hardwareMap.dcMotor.get("winch");

        //Servos
        claw = hardwareMap.servo.get("claw");

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        pivotServo.setPosition(0.027);
        hanging.setPosition(0);

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

            switch (Math.abs(liftState) % 5){
                case 0:
                    target = 78;
                    break;
                case 1:
                    target = 154;
                    break;
                case 2:
                    target = 332;
                    break;
                case 3:
                    target = 581;
                    break;
                case 4:
                    target = 778;
                    break;
            }

            if (liftUp.wasJustPressed()) {
                if(Math.abs(liftState) < 4){
                    liftState ++;
                }
            } else if (liftDown.wasJustPressed()) {
                if (Math.abs(liftState) > 0){
                    liftState --;
                }
            }

//            power = Math.min(1.0, Math.max(0.2, Math.abs(target - arm.getCurrentPosition()) / 500.0));
            arm.setTargetPosition(target);
            arm.set(0.05);


//            if (gamepad2.left_trigger > 0.1){
//                lift.setPower(gamepad2.left_trigger);
//            } else if (0.5 * gamepad2.right_trigger > 0.1){
//                lift.setPower(-0.5 * gamepad2.right_trigger);
//            } else {
//                lift.setPower(0);
//            }

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
                claw.setPosition(0.3);
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
                    pivotServo.setPosition(0.65);
                    break;
            }

            liftUp.readValue();
            liftDown.readValue();

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("lift", arm.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.addData("At Target?", arm.atTargetPosition());
            telemetry.addData("liftState", liftState);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
            /**
             * High Chamber: -778
             * Low Chamber: -332
             * Low Basket: -581
             * Pick Up Sub: -154
             * Pick Up wall: -78
 */