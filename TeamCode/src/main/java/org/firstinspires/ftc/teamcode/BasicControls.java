package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import parts.MintWheels;

@TeleOp
public class BasicControls extends LinearOpMode {
    @Override
    public void runOpMode() {
        //These are the names on the driver hub
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        Servo testServo = hardwareMap.get(Servo.class, "test_servo");

        MintWheels drive = new MintWheels(hardwareMap, gamepad1, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            double powerLimit = 0.6;
            if (gamepad1.right_bumper) {
                powerLimit = 1;
            }
            if (gamepad1.left_bumper) {
                powerLimit = 0.25;
            }

            drive.runAny(axial, lateral, yaw, powerLimit);
        }
    }
}
