package org.firstinspires.ftc.teamcode;

public class MotorCalibrator {
    private final double targetAccel, torqueFrac;
    private double mult;

    private enum State {Init, Calibrating, Calibrated}

    private State s;
    public Accelerometer accel;

    public MotorCalibrator(double targetAccel, double torqueFrac) {
        this.targetAccel = targetAccel;
        this.torqueFrac = torqueFrac;
        s = State.Init;
        mult = 1;
        accel = new Accelerometer(100);
    }

    public double getTorqueFrac() {
        return mult * torqueFrac;
    }

    public double getAccel() {
        return accel.getAccel();
    }

    public void sample(double degrees) {
        if (s == State.Calibrating) {
            accel.sample(degrees);
        }
    }

    public void setActive(boolean a) {
        if (s == State.Init) {
            if (a) {
                s = State.Calibrating;
            }
        } else if (s == State.Calibrating) {
            if (!a) {
                calibrate();
                s = State.Calibrated;
            }
        }
    }

    private void calibrate() {
        double m = targetAccel / accel.getAccel();
        if (Math.abs(m - 1) < 0.05)
            mult = m;
    }
}
