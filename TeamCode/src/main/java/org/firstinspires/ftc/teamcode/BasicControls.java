package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class BasicControls extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // max power of any motor, either direction
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            double powerLimit = 0.60;
            if (gamepad1.right_bumper)
                powerLimit = 1.0;

            if (max > powerLimit) {
                leftFrontPower *= powerLimit / max;
                rightFrontPower *= powerLimit / max;
                leftBackPower *= powerLimit / max;
                rightBackPower *= powerLimit / max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }
}
