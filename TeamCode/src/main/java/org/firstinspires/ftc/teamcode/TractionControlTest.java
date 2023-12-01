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

        MotorControl[] motors = new MotorControl[] {
                new MotorControl(leftFront, MotorConfig.driveMotor, getVs()),
                new MotorControl(rightFront, MotorConfig.driveMotor, getVs()),
                new MotorControl(leftBack, MotorConfig.driveMotor, getVs()),
                new MotorControl(rightBack, MotorConfig.driveMotor, getVs())
        };

        waitForStart();

        for (MotorControl m : motors) {
            m.setTargetAccel(1);
        }

        while (opModeIsActive()) {
            for (MotorControl m : motors) {
                m.run();
                telemetry.addData("speed",m.getSpeed());
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
