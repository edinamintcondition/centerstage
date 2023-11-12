package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class MotorEncTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");

        leftFront.setMode(RUN_TO_POSITION);
        rightFront.setMode(RUN_TO_POSITION);
        leftBack.setMode(RUN_TO_POSITION);
        rightBack.setMode(RUN_TO_POSITION);

        int targetPos = 3;

        waitForStart();

        while (opModeIsActive()){
            leftFront.setTargetPosition(targetPos);
            rightFront.setTargetPosition(targetPos);
            leftBack.setTargetPosition(targetPos);
            rightBack.setTargetPosition(targetPos);
        }
    }
}