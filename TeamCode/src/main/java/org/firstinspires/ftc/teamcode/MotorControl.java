package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorControl {
    private final MotorConfig mConf;
    private final DcMotor m;
    private final VoltageSensor voltSense;
    private double torqueFrac, prevSpeed;
    private final Speedometer s;
    private final ElapsedTime timer;
    private final LinearFuncFitter modelFitter;
    private LinearFunc model;
    private double targetAccel;
    private double nextTargetAccel;

    public MotorControl(DcMotor m, MotorConfig mConf, VoltageSensor voltSense) {
        this.mConf = mConf;
        this.m = m;
        this.voltSense = voltSense;
        s = new Speedometer(4);
        timer = new ElapsedTime();
        modelFitter = new LinearFuncFitter(8);
    }

    public void setTargetAccel(double a) {
        nextTargetAccel = a;
    }

    public double getSpeed() {
        return prevSpeed;
    }

    public void run() {
        double motorCurDeg = mConf.toDeg(m.getCurrentPosition());
        s.sample(motorCurDeg);
        double currSpeed = s.getSpeed();

        double volt = (torqueFrac + currSpeed / mConf.topSpeed) * mConf.nominalVolt;
        m.setPower(volt / voltSense.getVoltage());

        if (timer.milliseconds() < 100) {
            return;
        }

        double accel = (currSpeed - prevSpeed) / timer.seconds();

        boolean useModel;
        if (model != null) {
            useModel = Math.abs(accel - targetAccel) < 1;
        } else {
            useModel = false;
        }

        addToModel(accel, torqueFrac);

        if (Math.abs(currSpeed) > 200)
            nextTargetAccel = 0;

        targetAccel = nextTargetAccel;
        prevSpeed = currSpeed;

        if (useModel) {
            torqueFrac = model.eval(targetAccel);
        } else {
            double z = Math.random() * 0.01;
            torqueFrac = (torqueFrac + z) * .5;
        }

        timer.reset();
    }

    private void addToModel(double accel, double torqueFrac) {
        if (in(accel, modelFitter.getXData(), 2) && in(torqueFrac, modelFitter.getYData(), 0.001))
            return;

        modelFitter.sample(accel, torqueFrac);
        LinearFunc model = modelFitter.fit();

        if (model != null) {
            if (Math.abs(model.R2) > 0.3) {
                this.model = model;
            }
        }
    }

    private boolean in(double x, Iterable<Double> seq, double tol) {
        for (double y : seq) {
            if (Math.abs(x - y) < tol)
                return true;
        }

        return false;
    }
}