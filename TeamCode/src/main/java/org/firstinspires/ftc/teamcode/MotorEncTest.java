package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class MotorEncTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        double ppi= 537.0*5/60.875;

        int targetDist = 30;

        waitForStart();


        leftFront.setMode(STOP_AND_RESET_ENCODER);
        rightFront.setMode(STOP_AND_RESET_ENCODER);
        leftBack.setMode(STOP_AND_RESET_ENCODER);
        rightBack.setMode(STOP_AND_RESET_ENCODER);

        telemetry.addData("c pos", "%d %d %d %d", leftFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightFront.getCurrentPosition(), rightBack.getCurrentPosition());
        telemetry.update();

        int targetPos = (int)(targetDist*ppi);

        leftFront.setTargetPosition(targetPos);
        rightFront.setTargetPosition(targetPos);
        leftBack.setTargetPosition(targetPos);
        rightBack.setTargetPosition(targetPos);

        leftFront.setPower(.3);
        rightFront.setPower(.3);
        leftBack.setPower(.3);
        rightBack.setPower(.3);

        leftFront.setMode(RUN_TO_POSITION);
        rightFront.setMode(RUN_TO_POSITION);
        leftBack.setMode(RUN_TO_POSITION);
        rightBack.setMode(RUN_TO_POSITION);

        while (opModeIsActive()) {
            if (!leftFront.isBusy()) {
                break;
            }
        }
    }
}