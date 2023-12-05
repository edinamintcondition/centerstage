package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@SuppressLint("DefaultLocale")
public class MotorCalMode extends LinearOpMode {
    private VoltageSensor vs;
    private final MotorConfig mConf = MotorConfig.driveMotor;

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

        double accelVolts = 1.26;
        double torqueFrac = accelVolts / 12;

        TractionControl tc = new TractionControl(motors, vs);
        waitForStart();

        telemetry.addData("test", "motor starting voltage");
        telemetry.update();

        double startVolts = testStartVolts(motors);

        telemetry.addData("test", "motor acceleration");
        telemetry.addData("v start", "%.2f", startVolts);
        telemetry.update();

        double[] aScale = new double[]{1, 1, 1, 1};
        double[] avgAccel = new double[4];
        int nTest = 6;
        double tgtSpeed = 400;
        for (int test = 0; test < nTest; test++) {
            double[] accel = testAccel(tc, tgtSpeed);
            for (int i = 0; i < 4; i++)
                avgAccel[i] += Math.abs(accel[i]) / nTest;
            dump(motors, accelVolts, accel);

            sleep(1000);

            tgtSpeed = -tgtSpeed;
        }

        for (int i = 0; i < 4; i++)
            telemetry.addData("accel" + i, "%.2f", avgAccel[i], avg(avgAccel) / avgAccel[i]);
        telemetry.update();

        sleep(1000);

        testSpeed(tc, tgtSpeed / 2);

        while (opModeIsActive()) {
            sleep(1);
        }
    }

    private double testStartVolts(DcMotor[] m) {
        double[] initPos = new double[4];
        for (int i = 0; i < 4; i++) {
            m[i].setMode(RUN_WITHOUT_ENCODER);
            initPos[i] = m[i].getCurrentPosition();
        }

        double volt = 0;
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            int moved = 0;
            for (int i = 0; i < 4; i++) {
                if (Math.abs(m[i].getCurrentPosition() - initPos[i]) > 4)
                    moved++;
            }

            if (moved == 4)
                break;

            volt = t.seconds() / 2;
            if (volt > 12) throw new RuntimeException("motor did not start");

            telemetry.addData("volt", "%.02f", volt);
            telemetry.update();

            for (int i = 0; i < 4; i++)
                m[i].setPower(volt / vs.getVoltage());
        }

        for (int i = 0; i < 4; i++)
            m[i].setPower(0);
        sleep(50);

        return volt;
    }

    private double[] testAccel(TractionControl tc, double tgtSpeed) {
        Accelerometer[] a = new Accelerometer[4];
        for (int i = 0; i < 4; i++)
            a[i] = new Accelerometer(10000);

        tc.setTargetDriveSpeed(tgtSpeed);

        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++)
                a[i].sample(tc.get(0).getDeg());

            tc.run();

            for (int i = 0; i < 4; i++)
                telemetry.addData("motor" + i, tc.get(i));

            telemetry.addData("speed", "%.2f", tc.getDriveSpeed());

            if (Math.abs(tc.getDriveSpeed() - tgtSpeed) < 10)
                break;

            telemetry.update();
        }

        double[] totalAccel = new double[4];
        for (int i = 0; i < 4; i++) {
            totalAccel[i] = a[i].getAccel();
            telemetry.addData("accel" + i, "%.2f", totalAccel[i]);
        }

        telemetry.update();

        stop(tc);

        return totalAccel;
    }

    private double[] testSpeed(TractionControl tc, double tgtSpeed) {
        Accelerometer[] a = new Accelerometer[4];
        for (int i = 0; i < 4; i++)
            a[i] = new Accelerometer(10000);

        tc.setTargetDriveSpeed(tgtSpeed);

        while (opModeIsActive()) {
            tc.run();
            if (Math.abs(tc.getDriveSpeed() - tgtSpeed) < 10)
                break;
        }

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++)
                a[i].sample(tc.get(0).getDeg());

            tc.run();

            if (t.milliseconds() > 500)
                break;
        }

        double[] totalAccel = new double[4];
        for (int i = 0; i < 4; i++) {
            totalAccel[i] = a[i].getAccel();
            telemetry.addData("accel" + i, "%.2f", totalAccel[i]);
        }

        telemetry.update();

        stop(tc);

        return totalAccel;
    }

    private void speedTest(DcMotor[] m, double[] volts) {
        Speedometer[] s = new Speedometer[4];
        Accelerometer[] a = new Accelerometer[4];
        for (int i = 0; i < 4; i++) {
            s[i] = new Speedometer(25);
            a[i] = new Accelerometer(25);
            m[i].setMode(RUN_WITHOUT_ENCODER);
            m[i].setPower(volts[i] / vs.getVoltage());
        }

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                double motorCurrDeg = mConf.toDeg(m[i].getCurrentPosition());
                s[i].sample(motorCurrDeg);
                a[i].sample(motorCurrDeg);

                telemetry.addData("accel" + i, "%.1f", a[i].getAccel());
            }

            if (t.milliseconds() > 1500)
                break;

            telemetry.update();
        }

        stopAll(m);

        double[] speed = new double[4];
        for (int i = 0; i < 4; i++) {
            speed[i] = s[i].getSpeed();
            telemetry.addData("speed" + i, "%.1f", s[i].getSpeed());
        }

        telemetry.update();
    }

    private void stopAll(DcMotor[] m) {
        for (int i = 0; i < 4; i++) {
            m[i].setZeroPowerBehavior(BRAKE);
            m[i].setMode(RUN_USING_ENCODER);
            m[i].setPower(0);
        }
    }

    private void stop(TractionControl tc) {
        for (int i = 0; i < 4; i++)
            tc.get(i).setTargetSpeed(0);

        while (opModeIsActive()) {
            if (Math.abs(tc.getDriveSpeed()) < 10)
                break;

            tc.run();
        }
    }

    private void dump(DcMotor[] motors, double startVolts, double[] accel) {
        telemetry.addData("v start", "%.2f", startVolts);
        for (int i = 0; i < 4; i++) {
            double pos = mConf.toPos(motors[i].getCurrentPosition());
            String s = String.format("p=%.2f, a=%.2f", pos, accel[i]);
            telemetry.addData("motor" + i, s);
        }
        telemetry.update();
    }

    private void getVs() {
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            this.vs = vs;
            return;
        }

        throw new RuntimeException("no voltage sensor");
    }

    private void rescale(double[] scale, double[] accel) {
        double sum = 0;
        for (int i = 0; i < 4; i++)
            sum += accel[i];
        double avg = sum / 4;

        for (int i = 0; i < 4; i++)
            scale[i] *= avg / accel[i];
    }

    private double avg(double[] x) {
        double sum = 0;
        for (int i = 0; i < x.length; i++)
            sum += x[i];
        return sum / x.length;
    }
}
