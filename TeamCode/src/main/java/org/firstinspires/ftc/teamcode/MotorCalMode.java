package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import parts.MintDrive;

@Disabled
@Autonomous
@SuppressLint("DefaultLocale")
public class MotorCalMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        MintDrive tc = new MintDrive(hardwareMap,telemetry);
        waitForStart();

        telemetry.addData("test", "motor starting voltage");
        telemetry.update();

//        double startVolts = testStartVolts(tc.getMotors());
//        telemetry.addData("test", "motor acceleration");
//        telemetry.addData("v start", "%.2f", startVolts);
//        telemetry.update();
//        sleep(2000);

        double[] avgAccel = new double[4];
        int nTest = 3;
        boolean strafe = false;
        double tgtSpeed = 300;
        for (int test = 0; test < nTest; test++) {
            double[] accel = testAccel(tc, tgtSpeed, strafe);
            for (int i = 0; i < 4; i++)
                avgAccel[i] += Math.abs(accel[i]) / nTest;

            sleep(1000);

            testAccel(tc, -tgtSpeed, strafe);
            sleep(1000);
        }

        for (int i = 0; i < 4; i++)
            telemetry.addData("accel" + i, "a = %.2f, mult = %.4f", avgAccel[i], avg(avgAccel) / avgAccel[i]);
        telemetry.addData("average accel", "%.4f", avg(avgAccel));
        telemetry.update();

        sleep(4000);

        double[] avgDeccel = new double[4];
        for (int test = 0; test < nTest; test++) {
            double[] deccel = testDeccel(tc, tgtSpeed, strafe);
            for (int i = 0; i < 4; i++)
                avgDeccel[i] += Math.abs(deccel[i]) / nTest;

            sleep(1000);

            testDeccel(tc, -tgtSpeed, strafe);
            sleep(1000);
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

    private double[] testAccel(MintDrive tc, double tgtSpeed, boolean strafe) {
        Accelerometer[] a = new Accelerometer[4];
        for (int i = 0; i < 4; i++)
            a[i] = new Accelerometer(10000);

        tc.setTargetSpeed(tgtSpeed, strafe);

        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++)
                a[i].sample(tc.get(i).getDeg());

            tc.run(strafe);

            for (int i = 0; i < 4; i++)
                telemetry.addData("motor" + i, tc.get(i));
            telemetry.addData("speed", "%.2f", tc.getSpeed(strafe));
            telemetry.update();

            if (Math.abs(tc.getSpeed(strafe) - tgtSpeed) < 10)
                break;
        }

        double[] accel = new double[4];
        for (int i = 0; i < 4; i++)
            accel[i] = a[i].getAccel();

        stop(tc, strafe);

        return accel;
    }

    private double[] testDeccel(MintDrive tc, double tgtSpeed, boolean strafe) {
        Accelerometer[] a = new Accelerometer[4];
        for (int i = 0; i < 4; i++)
            a[i] = new Accelerometer(10000);

        tc.setTargetSpeed(tgtSpeed, strafe);

        while (opModeIsActive()) {
            tc.run(strafe);

            if (Math.abs(tc.getSpeed(strafe) - tgtSpeed) < 10)
                break;
        }

        tc.setTargetSpeed(0, strafe);

        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++)
                a[i].sample(tc.get(i).getDeg());

            tc.run(strafe);

            if (tc.isStopped())
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

    private void stop(MintDrive tc, boolean strafe) {
        for (int i = 0; i < 4; i++)
            tc.get(i).setTargetSpeed(0);

        while (opModeIsActive()) {
            tc.run(strafe);

            for (int i = 0; i < 4; i++)
                telemetry.addData("motor" + i, tc.get(i));
            telemetry.update();

            if (tc.isStopped())
                break;
        }
    }

    private double avg(double[] x) {
        double sum = 0;
        for (double v : x) sum += v;
        return sum / x.length;
    }
}
