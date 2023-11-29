package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class MotorTestMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        VoltageSensor vs = getVs();

        waitForStart();

        DcMotor m = null;
        MotorConfig mc = null;

        while (opModeIsActive()) {
            telemetry.addData("name", vs.getDeviceName());
            telemetry.addData("V", vs.getVoltage());
            telemetry.update();

            if (gamepad1.x) {
                m = hardwareMap.get(DcMotor.class, "front_left_motor");
                mc = MotorConfig.driveMotor;
            }
            if (gamepad1.y) {
                m = hardwareMap.get(DcMotor.class, "back_left_motor");
            }
            if (gamepad1.a) {
                m = hardwareMap.get(DcMotor.class, "front_right_motor");
            }
            if (gamepad1.b) {
                m = hardwareMap.get(DcMotor.class, "back_right_motor");
            }
            if (gamepad1.dpad_up) {
                m = hardwareMap.get(DcMotor.class, "arm_motor");
            }

            if (m != null) {
                break;
            }
        }

        while (opModeIsActive()) {
            m.setPower(0);
            // run to position = 360 deg
            // sleep 1000
        }

        while (opModeIsActive()) {
            // set 12v power
            // convert position to degrees
            // put degrees in speedometer
            // display speed in telemetry
        }
    }

    private VoltageSensor getVs() {
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            return vs;
        }

        throw new RuntimeException("no voltage sensor");
    }
}