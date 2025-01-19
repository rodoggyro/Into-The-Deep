package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class MotorTesting extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        waitForStart();

        while (opModeIsActive()){
            frontLeft.setPower(gamepad1.left_trigger);
            frontRight.setPower(gamepad1.right_trigger);
            backLeft.setPower(Math.abs(gamepad1.left_stick_y));
            backRight.setPower(Math.abs(gamepad1.right_stick_y));

            telemetry.addData("fr power", frontRight.getPower());
            telemetry.update();
        }
    }
}
