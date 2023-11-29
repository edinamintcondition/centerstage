package org.firstinspires.ftc.teamcode;

public class Speedometer {
    private final LinearFuncFitter fitter;

    public Speedometer(int numSamples) {
        fitter = new LinearFuncFitter(numSamples);
    }

    private double speed;

    public double getSpeed() {
        return speed;
    }

    public void sample(double t, double x) {
        fitter.sample(t, x);
        LinearFunc fit = fitter.fit();
        if (fit != null) {
            speed = fit.beta;
        }
    }
}
