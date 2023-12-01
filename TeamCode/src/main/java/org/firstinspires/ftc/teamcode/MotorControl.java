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

    public MotorControl(DcMotor m, MotorConfig mConf, VoltageSensor voltSense) {
        this.mConf = mConf;
        this.m = m;
        this.voltSense = voltSense;
        s = new Speedometer(4);
        timer = new ElapsedTime();
        modelFitter = new LinearFuncFitter(8);
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

        double currAccel = (currSpeed - prevSpeed) / timer.seconds();

        timer.reset();
    }

    private void addToModel(double accel, double torqueFrac) {
        if (in(accel, modelFitter.getXData(), 2) && in(torqueFrac, modelFitter.getYData(), 0.001))
            return;

        modelFitter.sample(accel, torqueFrac);
        model = modelFitter.fit();
    }

    private boolean in(double x, Iterable<Double> seq, double tol) {
        for (double y : seq) {
            if (Math.abs(x - y) < tol)
                return true;
        }

        return false;
    }
}