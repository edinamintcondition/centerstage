package parts;

import org.firstinspires.ftc.teamcode.Accelerometer;

public class DynamicParams {
    private final double targetAccel, maxSpeed;
    private double targetDeccel;
    private double accelMult, errMult, dragMult;
    private Accelerometer aMeter, cMeter, dMeter;
    private Calibrator accelMultCal, errMultCal;

    public DynamicParams(double targetAccel, double maxSpeed, double targetDeccel,
                         double accelMult, double errMult, double dragMult) {
//        accelMult = 0.000235201657558;
//        errMult = 0.0001;
//        dragMult = 0.000041653270461;
        this.targetAccel = targetAccel;
        this.maxSpeed = maxSpeed;
        this.targetDeccel = targetDeccel;
        this.accelMult = accelMult;
        this.errMult = errMult;
        this.dragMult = dragMult;

        aMeter = new Accelerometer(1000);
        cMeter = new Accelerometer(1000);
        dMeter = new Accelerometer(1000);

        accelMultCal = new Calibrator(targetAccel, accelMult);
        errMultCal = new Calibrator(0, errMult);
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

    public double getDragMult() {
        return dragMult;
    }

    public double getTargetAccel() {
        return targetAccel;
    }

    public double getTargetDeccel() {
        return targetDeccel;
    }

    public void sampleAccel(double deg) {
    }

    public void sampleCruise(double deg) {
    }

    public void sampleDeccel(double deg) {
    }

    public void update() {
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }
}
