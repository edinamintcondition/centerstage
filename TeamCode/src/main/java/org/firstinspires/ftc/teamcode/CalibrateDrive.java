package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous
public class CalibrateDrive extends LinearOpMode {

    DcMotor[] motors;

    @Override
    public void runOpMode() {
        int targetPos = 5000;
        double power = 0.1;

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};

        waitForStart();

        for (DcMotor m : motors) {
            m.setMode(STOP_AND_RESET_ENCODER);
        }

        sleep(100);

        for (DcMotor m : motors) {
            int p = m.getCurrentPosition();
            m.setTargetPosition(p + targetPos);
            m.setPower(power);
            m.setMode(RUN_TO_POSITION);
        }

        while (opModeIsActive()) {
            if (areIdle()) {
                telemetry.addData("Front Left", leftFront.getCurrentPosition());
                telemetry.addData("Front Right", rightFront.getCurrentPosition());
                telemetry.addData("Back Left", leftBack.getCurrentPosition());
                telemetry.addData("BackRight", rightBack.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    public boolean areIdle() {
        for (DcMotor m : motors) {
            if (m.isBusy()) {
                return false;
            }
        }
        return true;
    }
}
