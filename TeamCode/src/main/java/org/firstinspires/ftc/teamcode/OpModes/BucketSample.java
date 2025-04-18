package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="Bucket Sample")
@Disabled
public class BucketSample extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    Servo pivotServo;
    Servo claw;

    DcMotor lift; //-1000

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        pivotServo = hardwareMap.servo.get("pivotServo");
        pivotServo.setPosition(0.027);

        claw = hardwareMap.servo.get("claw");
        claw.setPosition(1);

        lift = hardwareMap.dcMotor.get("arm");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        lift.setTargetPosition(-1750);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-1);

        frontLeft.setPower(0.75);
        frontRight.setPower(0.75);
        backRight.setPower(0.75);
        backLeft.setPower(0.75);

        sleep(750);

        frontLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(0.5);

        sleep(1500);

        frontLeft.setPower(0.75);
        frontRight.setPower(0.75);
        backRight.setPower(0.75);
        backLeft.setPower(0.75);

        sleep(500);

        frontLeft.setPower(0.25);
        frontRight.setPower(0.25);
        backRight.setPower(0.25);
        backLeft.setPower(0.25);

        sleep(750);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        while (lift.isBusy()) {
            sleep(10);
        }

        frontLeft.setPower(0.25);
        frontRight.setPower(0.25);
        backRight.setPower(0.25);
        backLeft.setPower(0.25);

        sleep(350);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        claw.setPosition(0);

        sleep(500);

        frontLeft.setPower(-0.75);
        frontRight.setPower(-0.75);
        backRight.setPower(-0.75);
        backLeft.setPower(-0.75);


        sleep(500);

        lift.setTargetPosition(-1000);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-1);

        frontLeft.setPower(0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(0.5);
        backRight.setPower(-0.5);

        sleep(1350);

        frontLeft.setPower(0.75);
        frontRight.setPower(0.75);
        backRight.setPower(0.75);
        backLeft.setPower(0.75);

        sleep(750);

        frontLeft.setPower(0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(0.5);
        backRight.setPower(-0.5);

        sleep(750);

        frontLeft.setPower(0.75);
        frontRight.setPower(0.75);
        backRight.setPower(0.75);
        backLeft.setPower(0.75);

        sleep(500);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
//
//        pivotServo.setPosition(0.33);
//        sleep(500);
    }
}
