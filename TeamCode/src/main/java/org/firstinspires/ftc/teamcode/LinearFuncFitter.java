package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Iterator;

public class LinearFuncFitter {
    private final int numSamples;
    private final ArrayList<Double> xs, ys;

    public LinearFuncFitter(int numSamples) {
        this.numSamples = numSamples;
        xs = new ArrayList<Double>();
        ys = new ArrayList<Double>();
    }

    public Iterable<Double> getXData() {
        return xs;
    }

    public Iterable<Double> getYData() {
        return ys;
    }

    public void sample(double x, double y) {
        while (xs.size() >= numSamples) {
            xs.remove(0);
        }
        while (ys.size() >= numSamples) {
            ys.remove(0);
        }

        xs.add(x);
        ys.add(y);
    }

    public LinearFunc fit() {
        // find average
        int n = Math.min(xs.size(), ys.size());
        double xSum = 0.0;
        double ySum = 0.0;
        for (int i = 0; i < n; i++) {
            xSum += xs.get(i);
            ySum += ys.get(i);
        }

        double xMean = xSum / n;
        double yMean = ySum / n;

        // calculate variance

        double cov = 0.0;
        double xVar = 0.0;
        for (int i = 0; i < n; i++) {
            double xc = xs.get(i) - xMean;
            double yc = ys.get(i) - yMean;

            cov += xc * yc;
            xVar += xc * xc;
        }

        // do not divide by zero
        if (xVar == 0) {
            return null;
        }

        double beta = cov / xVar;
        double alpha = yMean - beta * xMean;

        //calculate R^2
        double rss = 0.0;
        double tss = 0.0;
        for (int i = 0; i < n; i++) {
            double yHat = beta * xs.get(i) + alpha;
            double res = ys.get(i) - yHat;
            double t = ys.get(i) - yMean;

            rss += res * res;
            tss += t * t;
        }

        double r2 = 1 - rss / tss;

        return new LinearFunc(beta, alpha, r2);
    }
}