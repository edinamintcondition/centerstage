package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
                mc = MotorConfig.driveMotor;
            }
            if (gamepad1.a) {
                m = hardwareMap.get(DcMotor.class, "front_right_motor");
                mc = MotorConfig.driveMotor;
            }
            if (gamepad1.b) {
                m = hardwareMap.get(DcMotor.class, "back_right_motor");
                mc = MotorConfig.driveMotor;
            }
            if (gamepad1.dpad_up) {
                m = hardwareMap.get(DcMotor.class, "arm_motor");
                mc = MotorConfig.armMotor;
            }

            if (m != null) {
                break;
            }
        }

        m.setMode(STOP_AND_RESET_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime t=new ElapsedTime();

        while (opModeIsActive()) {
            m.setTargetPosition(mc.toPos(360));
            m.setPower(1);
            m.setMode(RUN_TO_POSITION);

            if(t.milliseconds() > 1000) {
                break;
            }
        }

        Speedometer s = new Speedometer(8);
        m.setMode(RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            m.setPower(12 / vs.getVoltage());
            double motorCurDeg = mc.toDeg(m.getCurrentPosition());
            s.sample(motorCurDeg);

            telemetry.addData("speed", s.getSpeed());
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