package parts;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Speedometer;

@SuppressLint("DefaultLocale")
public class MintMotor {
    private final DcMotor motor;
    private final VoltageSensor vs;
    private final MotorConfig motorConf;
    private final double accelTorqueFrac;
    private final double cruiseTorqueFrac;
    private final Speedometer speedo;
    private double targetSpeed, initPos;
    private double torqueFrac, prevTorqueFrac;
    private double currTime, prevTime;
    private static final double speedTol = 20;
    private static final double stopTol = 30;
    private static final double coastToStopTol = 90;
    private final double torqueRamp;
    private boolean cruising;
    private final ElapsedTime t;

    public MintMotor(DcMotor motor, MotorConfig motorConf, VoltageSensor vs, double accelTf, double cruiseTf, double torqueRamp) {
        this.motor = motor;
        this.vs = vs;
        this.motorConf = motorConf;
        this.accelTorqueFrac = accelTf;
        this.cruiseTorqueFrac = cruiseTf;
        this.torqueRamp = torqueRamp;
        speedo = new Speedometer(8);
        t = new ElapsedTime();
    }

    public DcMotor getMotor() {
        return motor;
    }

    public double getDeg() {
        return motorConf.toDeg(motor.getCurrentPosition() - initPos);
    }

    public double getSpeed() {
        return speedo.getSpeed();
    }

    public boolean isStopped() {
        return Math.abs(getSpeed()) < stopTol;
    }

    public void setTargetSpeed(double t) {
        if (t != targetSpeed) {
            targetSpeed = t;
            cruising = false;
        }
    }

    public void resetDeg() {
        speedo.clearSamples();
        initPos = motor.getCurrentPosition();
    }

    public void sample() {
        speedo.sample(getDeg());
        prevTorqueFrac = torqueFrac;
        prevTime = currTime;
        currTime = t.seconds();
    }

    private static final double ACCEL_MULT = 0.000235201657558, ERR_MULT = 0.0001, DRAG_MULT = 0.000041653270461;

    public void run(double speed, double accelTgt, double degTgt, double dir) {
        double x = accelTgt == 0 ? 1 : 0;

        speed *= dir;
        accelTgt *= dir;
        degTgt *= dir;
        x *= dir;

        double degErr = degTgt - getDeg();

        double torqueFrac = ACCEL_MULT * accelTgt + ERR_MULT * degErr + x * DRAG_MULT;
        double volt = (torqueFrac + speed / motorConf.topSpeed) * motorConf.nominalVolt;
        motor.setPower(volt / vs.getVoltage());
    }

    public void run(double speed) {
        if (targetSpeed == 0) {
            if (Math.abs(speed) < coastToStopTol) {
                torqueFrac = 0;
                motor.setPower(0);
                return;
            }
        }

        double minTf, maxTf;
        if (cruising) {
            double deltaTime = currTime - prevTime;
            minTf = prevTorqueFrac - torqueRamp * deltaTime;
            maxTf = prevTorqueFrac + torqueRamp * deltaTime;
        } else {
            minTf = -1;
            maxTf = 1;
        }

        if (Math.abs(speed - targetSpeed) < speedTol) {
            int dir = targetSpeed < 0 ? -1 : 1;
            torqueFrac = dir * cruiseTorqueFrac;
            cruising = true;
        } else if (speed < targetSpeed) {
            torqueFrac = accelTorqueFrac;
        } else {
            torqueFrac = -accelTorqueFrac;
        }

        if (torqueFrac < minTf) torqueFrac = minTf;
        if (torqueFrac > maxTf) torqueFrac = maxTf;

        double volt = (torqueFrac + speed / motorConf.topSpeed) * motorConf.nominalVolt;
        motor.setPower(volt / vs.getVoltage());
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("tgt spd=%.2f, spd=%.2f, trq=%.2f, pwr=%.2f, %s", targetSpeed, getSpeed(), torqueFrac, motor.getPower(),
                cruising ? "c" : "a");
    }
}
