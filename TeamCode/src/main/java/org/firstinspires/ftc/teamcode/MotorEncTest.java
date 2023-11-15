package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

        DcMotor[] motors = new DcMotor[]{
                leftFront,
                leftBack,
                rightFront,
                rightBack
        };

        DcMotor[] fwdMotors = new DcMotor[]{
                leftFront,
                rightBack
        };

        DcMotor[] revMotors = new DcMotor[]{
                leftBack,
                rightFront
        };

        waitForStart();

        for (DcMotor m : motors) {
            m.setMode(STOP_AND_RESET_ENCODER);
        }

        strafeDistance(3.75, fwdMotors, revMotors);
        sleep(200);
        driveDistance(72, motors);
        sleep(200);
        strafeDistance(24, fwdMotors, revMotors);
        sleep(200);
        driveDistance(10, motors);
        sleep(200);
        driveDistance(-10, motors);
        sleep(200);
        rotateAmount(180, motors);
    }

    private void driveDistance(double targetDist, DcMotor[] motors) {
        double fastLimit = 10;
        double ppi = 537.0 * 5 / 60.875;

        int targetPos = (int) (targetDist * ppi);
        double power;

        if (targetDist > fastLimit) {
            power = 1;
        } else {
            power = .25;
        }

        for (DcMotor m : motors) {
            m.setTargetPosition(targetPos);
            m.setPower(power);
            m.setMode(RUN_TO_POSITION);
        }

        while (opModeIsActive()) {
            if (areIdle(motors)) {
                break;
            }
        }
    }

    private void rotateAmount(double targetAngle, DcMotor[] motors) {
        double ppd = 537.0 / 63.15;

        int targetPos = (int) (targetAngle * ppd);
        double power = 0.5;

        for (DcMotor m : motors) {
            if (m.getDirection() == DcMotorSimple.Direction.FORWARD) {
                m.setTargetPosition(-targetPos);
            } else {
                m.setTargetPosition(targetPos);
            }

            m.setPower(power);
            m.setMode(RUN_TO_POSITION);
        }

        while (opModeIsActive()) {
            if (areIdle(motors)) {
                break;
            }
        }
    }

    private void strafeDistance(double targetDist, DcMotor[] fwdMotors, DcMotor[] revMotors) {
        double fastLimit = 10;
        double ppi = 50.09;

        int targetPos = (int) (targetDist * ppi);
        double power;

        if (targetDist > fastLimit) {
            power = 1;
        } else {
            power = .3;
        }

        for (DcMotor m : fwdMotors) {
            m.setTargetPosition(targetPos);
            m.setPower(power);
            m.setMode(RUN_TO_POSITION);
        }

        for (DcMotor m : revMotors) {
            m.setTargetPosition(-targetPos);
            m.setPower(power);
            m.setMode(RUN_TO_POSITION);
        }

        while (opModeIsActive()) {
            if (areIdle(revMotors) && areIdle(fwdMotors)) {
                break;
            }
        }
    }

    private boolean areIdle(DcMotor[] motors) {
        for (DcMotor m : motors) {
            if (m.isBusy()) {
                return false;
            }
        }
        return true;
    }
}