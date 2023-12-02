package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorControl {
    private final MotorConfig mConf;
    private final DcMotor m;
    private final VoltageSensor voltSense;
    private double torqueFrac, prevSpeed;
    private final Speedometer s, accelometer;
    private final ElapsedTime timer;
    private final BilinearFuncFitter modelFitter;
    private final LinearFuncFitter modelFitter1;
    private BilinearFunc model;
    private LinearFunc model1;
    private int numAccelSamples, numCoastSamples;
    private double targetAccel, targetSpeed, accel;
    private final double maxAccel;

    public MotorControl(DcMotor m, MotorConfig mConf, VoltageSensor voltSense, double maxAccel) {
        this.mConf = mConf;
        this.m = m;
        this.voltSense = voltSense;
        this.maxAccel = maxAccel;
        s = new Speedometer(4);
        accelometer = new Speedometer(50);
        timer = new ElapsedTime();
        modelFitter = new BilinearFuncFitter(50);
        modelFitter.sample(0, 0, 0);

        modelFitter1 = new LinearFuncFitter(50);
    }

    public void setTargetSpeed(double speed) {
        targetSpeed = speed;
    }

    public double getActualSpeed() {
        return prevSpeed;
    }

    boolean oops;

    public void run() {
        double motorCurDeg = mConf.toDeg(m.getCurrentPosition());
        s.sample(motorCurDeg);
        double currSpeed = s.getSpeed();

        accelometer.sample(currSpeed);

        double volt = (torqueFrac + currSpeed / mConf.topSpeed) * mConf.nominalVolt;
        if (Math.abs(currSpeed) < 5 && Math.abs(torqueFrac) < 0.01) {
            volt *= 2;
        }
        m.setPower(volt / voltSense.getVoltage());

        if (accelometer.getNumSamples() < 10) {
            return;
        }

        accel = accelometer.getSpeed();

        if (model == null) {
            modelFitter.sample(accel, prevSpeed, torqueFrac);
            modelFitter1.sample(accel, torqueFrac);

            if (numAccelSamples < 10) {
                torqueFrac = 0.2;
                numAccelSamples++;
            } else if (numCoastSamples < 10) {
                torqueFrac = 0;
                numCoastSamples++;
            } else {
//                model = modelFitter.fit();
//                if (model == null) {
//                    throw new RuntimeException("oops");
//                }
                model1 = modelFitter1.fit();
                if (model1 == null) {
                    oops = true;
                    return;
                }
            }
        }

        if (model != null) {
            if (Math.abs(currSpeed - targetSpeed) < 5) {
                targetAccel = 0;
            } else if (currSpeed < targetSpeed) {
                targetAccel = maxAccel;
            } else {
                targetAccel = -maxAccel;
            }

            torqueFrac = model.eval(targetAccel, 0 * currSpeed);
            torqueFrac = model1.eval(targetAccel);
            torqueFrac = Math.min(Math.max(torqueFrac, -1), 1);
        }

        prevSpeed = currSpeed;

        timer.reset();
        accelometer.clearSamples();
    }

    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() {
        String s = String.format("ta=%.2f, a=%.2f, tf=%.2f, v=%.2f", targetAccel, accel, torqueFrac, getActualSpeed());
        if (model != null) {
            s = String.format("%s, βa=%.2g, βv=%.2g, α=%.2g", s, model.beta0, model.beta1, model.alpha);
        }

        s += modelFitter1;

        return s;
    }

//    private void addToModel(double accel, double torqueFrac) {
//        if (in(accel, modelFitter.getXData(), 2) && in(torqueFrac, modelFitter.getYData(), 0.001))
//            return;
//
//        modelFitter.sample(accel, torqueFrac);
//        BilinearFunc model = modelFitter.fit();
//
//        if (model != null) {
//            if (Math.abs(model.R2) > 0.3) {
//                this.model = model;
//            }
//        }
//    }

    private boolean in(double x, Iterable<Double> seq, double tol) {
        for (double y : seq) {
            if (Math.abs(x - y) < tol)
                return true;
        }

        return false;
    }
}