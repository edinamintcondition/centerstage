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
    private LinearFunc model1, accelModel, coastModel, deccelModel;
    private int numAccelSamples, numCoastSamples;
    private double targetAccel, targetSpeed, accel;
    private final double maxAccel;
    private double accelFrac, coastFrac, deccelFrac;
    private boolean canCalibrate;
    private final MotorCalibrator accelCal, coastCal, deccelCal;
    private MotorCalibrator activeCal;

    private enum State {Stopped, Accel, Coast, Deccel}

    private State state;
    private int direction;

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
        accelModel = new LinearFunc(0, 0, 0);

        modelFitter1 = new LinearFuncFitter(50);

        // should get from motor config
        accelFrac = 0.2;
        coastFrac = 0.05;
        deccelFrac = -0.15;

        state = State.Stopped;

        accelCal = new MotorCalibrator(maxAccel, 0.16);
        coastCal = new MotorCalibrator(0, 0.06);
        deccelCal = new MotorCalibrator(-maxAccel, -0.05);
    }

    public void setTargetSpeed(double speed) {
        targetSpeed = speed;
        if (Math.abs(speed) > getActualSpeed()) {
            if (speed > getActualSpeed()) {
                setState(State.Accel, 1);
            } else {
                setState(State.Accel, -1);
            }
        } else {
            if (speed < getActualSpeed()) {
                setState(State.Deccel, 1);
            } else {
                setState(State.Deccel, -1);
            }
        }
    }

    public double getActualSpeed() {
        return s.getSpeed();
    }

    boolean oops;

    public void run() {
        double motorCurrDeg = mConf.toDeg(m.getCurrentPosition());
        s.sample(motorCurrDeg);
        double currSpeed = s.getSpeed();

//        accelometer.sample(currSpeed);

        if (state == State.Accel && Math.abs(currSpeed - targetSpeed) < 10) {
            setState(State.Coast, direction);
        }
        if (state == State.Deccel && Math.abs(currSpeed) < 10) {
            setState(State.Stopped, direction);
        }

        if (activeCal != null) {
            activeCal.sample(motorCurrDeg);
            torqueFrac = activeCal.getTorqueFrac() * direction;
        } else {
            torqueFrac = 0;
        }

        double volt = (torqueFrac + currSpeed / mConf.topSpeed) * mConf.nominalVolt;
        if (Math.abs(currSpeed) < 5 && Math.abs(torqueFrac) < 0.01) {
            volt = 6 * Math.signum(volt);
        }
        m.setPower(volt / voltSense.getVoltage());

//        if (accelometer.getNumSamples() < 10) {
//            return;
//        }
//
//        accel = accelometer.getSpeed();
//
//        if (model == null) {
//            modelFitter.sample(accel, prevSpeed, torqueFrac);
//            modelFitter1.sample(accel, torqueFrac);
//
//            if (numAccelSamples < 10) {
//                torqueFrac = 0.2;
//                numAccelSamples++;
//            } else if (numCoastSamples < 10) {
//                torqueFrac = 0;
//                numCoastSamples++;
//            } else {
////                model = modelFitter.fit();
////                if (model == null) {
////                    throw new RuntimeException("oops");
////                }
//                model1 = modelFitter1.fit();
//                if (model1 == null) {
//                    oops = true;
//                    return;
//                }
//            }
//        }
//
//        if (model != null) {
//            if (Math.abs(currSpeed - targetSpeed) < 5) {
//                targetAccel = 0;
//            } else if (currSpeed < targetSpeed) {
//                targetAccel = maxAccel;
//            } else {
//                targetAccel = -maxAccel;
//            }
//
//            torqueFrac = model.eval(targetAccel, 0 * currSpeed);
//            torqueFrac = model1.eval(targetAccel);
//            torqueFrac = Math.min(Math.max(torqueFrac, -1), 1);
//        }
//
//        prevSpeed = currSpeed;
//
//        timer.reset();
//        accelometer.clearSamples();
    }

    private void setState(State s, int dir) {
        if (activeCal != null)
            accelCal.setActive(false);

        state = s;
        direction = dir;

        if (state == State.Accel) {
            accelCal.setActive(true);
            activeCal = accelCal;
        } else if (state == State.Coast) {
            coastCal.setActive(true);
            activeCal = coastCal;
        } else if (state == State.Deccel) {
            deccelCal.setActive(true);
            activeCal = deccelCal;
        } else {
            activeCal = null;
        }
    }

    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() {
        String s = String.format("tf=%.2f, v=%.2f, %s", torqueFrac, getActualSpeed(), state);
        s = String.format("%s, a=%.2g, atf=%.2g", s, accelCal.getAccel(), accelCal.getTorqueFrac());
        s = String.format("%s, ctf=%.2g", s, coastCal.getTorqueFrac());
        s = String.format("%s, dtf=%.2g", s, deccelCal.getTorqueFrac());

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