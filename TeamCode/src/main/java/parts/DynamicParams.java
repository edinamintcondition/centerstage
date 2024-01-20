package parts;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.Accelerometer;

@SuppressLint("DefaultLocale")
public class DynamicParams {
    private final double targetAccel, maxSpeed, errMult;
    private double accelMult, cruiseMult, targetDeccel;
    private final Accelerometer aMeter, cMeter, dMeter;
    private final Calibrator accelMultCal, cruiseMultCal;

    public DynamicParams(double targetAccel, double maxSpeed, double targetDeccel,
                         double accelMult, double errMult, double cruiseMult) {
        this.targetAccel = targetAccel;
        this.maxSpeed = maxSpeed;
        this.targetDeccel = targetDeccel;
        this.accelMult = accelMult;
        this.errMult = errMult;
        this.cruiseMult = cruiseMult;

        aMeter = new Accelerometer(1000);
        cMeter = new Accelerometer(1000);
        dMeter = new Accelerometer(1000);

        accelMultCal = new Calibrator(targetAccel, accelMult);
        cruiseMultCal = new Calibrator(0, cruiseMult);
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

    public double getCruiseMult() {
        return cruiseMult;
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
        cruiseMult = cruiseMultCal.getGuess();
    }

    public void finishCalibration() {
        accelMultCal.updateGuess(aMeter.getAccel());
        cruiseMultCal.updateGuess(cMeter.getAccel());
        targetDeccel = dMeter.getAccel();
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    @Override
    public String toString() {
        return String.format("amult=%f, cmult=%f, dec=%f", accelMult, cruiseMult, targetDeccel);
    }
}
