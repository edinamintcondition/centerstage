package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class CalibrateStrafe extends LinearOpMode {
    DcMotorEx[] motors, fwdMotors, revMotors;

    @Override
    public void runOpMode() {
        int targetPos = -5000;

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack};

        fwdMotors = new DcMotorEx[]{leftFront, rightBack};

        revMotors = new DcMotorEx[]{leftBack, rightFront};

        waitForStart();

        ElapsedTime timer=new ElapsedTime();

        for (DcMotor m : motors) {
            m.setMode(STOP_AND_RESET_ENCODER);
        }

        double power = 0.1;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            power = 0.1 * vs.getVoltage() / 14;
            break;
        }

        sleep(100);

        for (DcMotorEx m : fwdMotors) {
            int p = m.getCurrentPosition();
            m.setTargetPosition(p + targetPos);
            m.setVelocity(30, AngleUnit.DEGREES);
            m.setMode(RUN_TO_POSITION);
        }

        for (DcMotorEx m : revMotors) {
            int p = m.getCurrentPosition();
            m.setTargetPosition(p - targetPos);
            m.setVelocity(30, AngleUnit.DEGREES);
            m.setMode(RUN_TO_POSITION);
        }

        int i=0;
        while (opModeIsActive()) {
            telemetry.addData("rate", i / timer.seconds());
            telemetry.update();
            i++;

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