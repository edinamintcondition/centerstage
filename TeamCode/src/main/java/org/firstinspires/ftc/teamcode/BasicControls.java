package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
        Servo testServo = hardwareMap.get(Servo.class, "test_servo");

        //Sets motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            double max;

            //These turn a joystick input into a number
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            //Arm
            double angle = gamepad2.right_stick_y * 45;
            telemetry.addData("angle", angle);
            telemetry.addData("Current Position", armMotor.getCurrentPosition());
            telemetry.update();
            armMotor.setTargetPosition((int) angle);
            armMotor.setPower(1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /*double armPower = gamepad2.right_stick_y * 0.5;
            armMotor.setPower(armPower);*/

            //Max power of any motor, either direction
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

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

            //Applies the power to the motors
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            double pos = testServo.getPosition();
            if(gamepad2.x)
                testServo.setPosition(pos - 0.1);

            if(gamepad2.y)
                testServo.setPosition(pos + 0.1);
        }
    }
}
