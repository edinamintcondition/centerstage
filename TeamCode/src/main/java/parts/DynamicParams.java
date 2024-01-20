package parts;

import org.firstinspires.ftc.teamcode.Accelerometer;

public class DynamicParams {
    private final double targetAccel, maxSpeed, errMult;
    private double accelMult, coastMult, targetDeccel;
    private final Accelerometer aMeter, cMeter, dMeter;
    private final Calibrator accelMultCal, coastMultCal;

    public DynamicParams(double targetAccel, double maxSpeed, double targetDeccel,
                         double accelMult, double errMult, double coastMult) {
        this.targetAccel = targetAccel;
        this.maxSpeed = maxSpeed;
        this.targetDeccel = targetDeccel;
        this.accelMult = accelMult;
        this.errMult = errMult;
        this.coastMult = coastMult;

        aMeter = new Accelerometer(1000);
        cMeter = new Accelerometer(1000);
        dMeter = new Accelerometer(1000);

        accelMultCal = new Calibrator(targetAccel, accelMult);
        coastMultCal = new Calibrator(0, coastMult);
    }

    public double getDpi() {
        return 15;
    }

    public double getAccelMult() {
        return accelMult;
    }

    public double getErrMult() {
        return errMult;
    }

    public double getCoastMult() {
        return coastMult;
    }

    public double getTargetAccel() {
        return targetAccel;
    }

    public double getTargetDeccel() {
        return targetDeccel;
    }

    public void sampleAccel(double deg) {
        aMeter.sample(deg);
    }

    public void sampleCruise(double deg) {
        cMeter.sample(deg);
    }

    public void sampleDeccel(double deg) {
        dMeter.sample(deg);
    }

    public void startCalibration() {
        accelMult = accelMultCal.getGuess();
        coastMult = coastMultCal.getGuess();
    }

    public void finishCalibration() {
        accelMultCal.updateGuess(aMeter.getAccel());
        coastMultCal.updateGuess(cMeter.getAccel());
        targetDeccel = dMeter.getAccel();
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }
}
