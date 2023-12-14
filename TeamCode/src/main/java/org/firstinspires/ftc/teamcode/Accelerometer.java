package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import data.BilinearFunc;
import data.BilinearFuncFitter;

public class Accelerometer {
    private final BilinearFuncFitter fitter;

    private final ElapsedTime elapsedTime;

    public Accelerometer(int numSamples) {
        fitter = new BilinearFuncFitter(numSamples);
        elapsedTime = new ElapsedTime();
    }

    private double accel;

    public double getAccel() {
        return accel;
    }

    public int getNumSamples() {
        return fitter.getNumSamples();
    }

    public void sample(double degrees) {
        double t = elapsedTime.seconds();
        fitter.sample(t * t, t, degrees);
        BilinearFunc fit = fitter.fit(false);
        if (fit != null) {
            accel = 2 * fit.beta0;
        }
    }
}
