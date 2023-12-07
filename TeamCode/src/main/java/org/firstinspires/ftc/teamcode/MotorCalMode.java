package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

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
        sleep(2000);

        double[] avgAccel = new double[4];
        int nTest = 6;
        double tgtSpeed = 300;
        for (int test = 0; test < 6; test++) {
            double[] accel = testAccel(tc, tgtSpeed);
            if (tgtSpeed > 0)
                for (int i = 0; i < 4; i++)
                    avgAccel[i] += Math.abs(accel[i]) * 2 / nTest;

            sleep(1000);

            tgtSpeed = -tgtSpeed;
        }

        for (int i = 0; i < 4; i++)
            telemetry.addData("accel" + i, "a = %.2f, mult = %.4f", avgAccel[i], avg(avgAccel) / avgAccel[i]);
        telemetry.addData("average accel", "%.4f", avg(avgAccel));
        telemetry.update();

        sleep(4000);

        double[] avgDeccel = new double[4];
        for (int test = 0; test < nTest; test++) {
            double[] deccel = testDeccel(tc, tgtSpeed);
            if (tgtSpeed > 0)
                for (int i = 0; i < 4; i++)
                    avgDeccel[i] += Math.abs(deccel[i]) * 2 / nTest;

            sleep(1000);

            tgtSpeed = -tgtSpeed;
        }

        for (int i = 0; i < 4; i++)
            telemetry.addData("deccel" + i, "a = %.2f, mult = %.4f", avgDeccel[i], avg(avgDeccel) / avgDeccel[i]);
        telemetry.addData("average deccel", "%.4f", avg(avgDeccel));
        telemetry.update();

        sleep(4000);

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

            volt = t.seconds() / 5;
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
                a[i].sample(tc.get(i).getDeg());

            tc.run();

            for (int i = 0; i < 4; i++)
                telemetry.addData("motor" + i, tc.get(i));
            telemetry.addData("speed", "%.2f", tc.getDriveSpeed());
            telemetry.update();

            if (Math.abs(tc.getDriveSpeed()) > Math.abs(tgtSpeed))
                break;
        }

        double[] accel = new double[4];
        for (int i = 0; i < 4; i++)
            accel[i] = a[i].getAccel();

        stop(tc);

        return accel;
    }

    private double[] testDeccel(TractionControl tc, double tgtSpeed) {
        Accelerometer[] a = new Accelerometer[4];
        for (int i = 0; i < 4; i++)
            a[i] = new Accelerometer(10000);

        tc.setTargetDriveSpeed(tgtSpeed);

        while (opModeIsActive()) {
            tc.run();

            if (Math.abs(tc.getDriveSpeed() - tgtSpeed) < 10)
                break;
        }

        tc.setTargetDriveSpeed(0);

        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++)
                a[i].sample(tc.get(i).getDeg());

            tc.run();

            if (Math.abs(tc.getDriveSpeed()) < 10)
                break;
        }

        double[] deccel = new double[4];
        for (int i = 0; i < 4; i++) {
            deccel[i] = a[i].getAccel();
            telemetry.addData("accel" + i, "%.2f", deccel[i]);
        }

        telemetry.update();

        return deccel;
    }

    private void stop(TractionControl tc) {
        for (int i = 0; i < 4; i++)
            tc.get(i).setTargetSpeed(0);

        while (opModeIsActive()) {
            tc.run();

            for (int i = 0; i < 4; i++)
                telemetry.addData("motor"+i,tc.get(i));
            telemetry.update();

            if (tc.isStopped())
                break;
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

    private double avg(double[] x) {
        double sum = 0;
        for (double v : x) sum += v;
        return sum / x.length;
    }
}
