package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

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
        getVs();

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

        telemetry.addData("test", "motor starting voltage");
        telemetry.update();

        double[] startVolts = new double[4];
        double maxVolts = 0;
        for (int i = 0; i < 4; i++) {
            startVolts[i] = testStartVolts(motors[i]);
            maxVolts = Math.max(startVolts[i], maxVolts);
        }

        telemetry.addData("test", "motor acceleration");
        telemetry.update();

        double[] accel = testAccel(motors, MotorConfig.driveMotor, 1.1 * maxVolts / 12);

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
        motor.setMode(RUN_WITHOUT_ENCODER);
        motor.setPower(0);

        double initPos = motor.getCurrentPosition();

        ElapsedTime t = new ElapsedTime();
        double volt = 0;
        while (opModeIsActive()) {
            if (Math.abs(motor.getCurrentPosition() - initPos) > 4)
                break;

            volt = t.seconds();
            if (volt > 12) throw new RuntimeException("motor did not start");

            telemetry.addData("power", volt / vs.getVoltage());
            telemetry.update();

            motor.setPower(volt / vs.getVoltage());
        }

        motor.setPower(0);
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
        for (int i = 0; i < m.length; i++) {
            totalAccel[i] = a[i].getAccel();
            m[i].setZeroPowerBehavior(BRAKE);
            m[i].setPower(0);
        }

        return totalAccel;
    }

    private void getVs() {
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            this.vs = vs;
            return;
        }

        throw new RuntimeException("no voltage sensor");
    }
}
