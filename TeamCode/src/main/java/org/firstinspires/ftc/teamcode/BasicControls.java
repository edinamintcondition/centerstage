package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BasicControls extends LinearOpMode {
    @Override
    public void runOpMode() {
        //These are the names on the driver hub
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        //Sets motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            double max;
            double armMax;

            //These turn a joystick input into a number
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double angle = gamepad2.left_stick_y;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            double armPower = angle;

            // max power of any motor, either direction
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            armMax = Math.max(max, Math.abs(armPower));

            //Sets power to 60% unless RB is pressed on game pad 1
            double powerLimit = 0.60;
            if (gamepad1.right_bumper)
                powerLimit = 1.0;
            if (max > powerLimit) {
                leftFrontPower *= powerLimit / max;
                rightFrontPower *= powerLimit / max;
                leftBackPower *= powerLimit / max;
                rightBackPower *= powerLimit / max;
            }

            //Sets arm power to 75% unless RB is pressed on game pad 2
            double armPowerLimit = 0.60;
            if (gamepad2.right_bumper)
                armPowerLimit = 0.8;
            if (armMax > armPowerLimit) {
                armPower *= armPowerLimit / max;
            }

            //Applies the power to the motors
            armMotor.setPower(armPower);
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }
}
