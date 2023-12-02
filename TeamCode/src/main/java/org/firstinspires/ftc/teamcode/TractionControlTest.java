package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous
public class TractionControlTest extends LinearOpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        leftFront.setDirection(FORWARD);

        rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        rightFront.setDirection(REVERSE);

        leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        leftBack.setDirection(FORWARD);

        rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");
        rightBack.setDirection(REVERSE);

        MotorControl[] motors = new MotorControl[]{
                new MotorControl(leftFront, MotorConfig.driveMotor, getVs(), 10),
                new MotorControl(rightFront, MotorConfig.driveMotor, getVs(), 10),
                new MotorControl(leftBack, MotorConfig.driveMotor, getVs(), 10),
                new MotorControl(rightBack, MotorConfig.driveMotor, getVs(), 10)
        };

        waitForStart();

        for (MotorControl m : motors) {
            m.setTargetSpeed(90);
        }

        while (opModeIsActive()) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].run();
                telemetry.addData("motor" + i, motors[i]);
            }

            telemetry.update();
        }
    }

    private VoltageSensor getVs() {
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            return vs;
        }

        throw new RuntimeException("no voltage sensor");
    }
}
