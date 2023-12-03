package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class MotorCalMode extends LinearOpMode {
    private VoltageSensor vs;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        VoltageSensor vs = getVs();

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] motors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};

        waitForStart();

        double[] startVolts = new double[4];
        double maxVolts = 0;
        for (int i = 0; i < 4; i++) {
            startVolts[i] = testStartVolts(motors[i]);
            if (startVolts[i] > maxVolts)
                maxVolts = startVolts[i];
        }

        double[] accel = testAccel(motors, MotorConfig.driveMotor, maxVolts / 12);

        for (int i = 0; i < 4; i++) {
            String s = String.format("v start=%.2f, a=%.2f", startVolts[i], accel[i]);
            telemetry.addData("motor" + i, s);
        }
        telemetry.update();

        while (opModeIsActive()) {
            sleep(1);
        }
    }

    private double testStartVolts(DcMotor motor) {
        motor.setMode(STOP_AND_RESET_ENCODER);
        sleep(50);

        ElapsedTime t = new ElapsedTime();
        double volt = 0;
        while (opModeIsActive()) {
            if (Math.abs(motor.getCurrentPosition()) > 4)
                break;

            volt = t.seconds();
            if (volt > 12) throw new RuntimeException("motor did not start");

            motor.setPower(volt / vs.getVoltage());
        }

        motor.setMode(STOP_AND_RESET_ENCODER);
        sleep(50);

        return volt;
    }

    private double[] testAccel(DcMotor[] m, MotorConfig mConf, double torqueFrac) {
        Speedometer[] s = new Speedometer[m.length];
        Accelerometer[] a = new Accelerometer[m.length];
        for (int i = 0; i < m.length; i++) {
            s[i] = new Speedometer(4);
            a[i] = new Accelerometer(10000);
        }

        boolean done = false;
        while (opModeIsActive()) {
            for (int i = 0; i < m.length; i++) {
                double motorCurrDeg = mConf.toDeg(m[i].getCurrentPosition());
                s[i].sample(motorCurrDeg);
                a[i].sample(motorCurrDeg);

                double currSpeed = s[i].getSpeed();
                if (currSpeed > 0.75 * mConf.topSpeed) {
                    done = true;
                    break;
                }

                double volt = (torqueFrac + currSpeed / mConf.topSpeed) * mConf.nominalVolt;
                m[i].setPower(volt / vs.getVoltage());
            }

            if (done)
                break;
        }

        double[] totalAccel = new double[m.length];
        for (int i = 0; i < m.length; i++)
            totalAccel[i] = a[i].getAccel();

        return totalAccel;
    }

    private VoltageSensor getVs() {
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            return vs;
        }

        throw new RuntimeException("no voltage sensor");
    }
}
