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
    private static final double speedTol = 30;
    private static final double coastToStopTol = 90;
    private final double torqueRamp;
    private final ElapsedTime t;

    public MintMotor(DcMotor motor, MotorConfig motorConf, VoltageSensor vs, double accelTf, double coastTf, double torqueRamp) {
        this.motor = motor;
        this.vs = vs;
        this.motorConf = motorConf;
        this.accelTorqueFrac = accelTf;
        this.cruiseTorqueFrac = coastTf;
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
        return Math.abs(getSpeed()) < speedTol;
    }

    public void setTargetSpeed(double t) {
        targetSpeed = t;
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

    public void run(double speed) {
        if (targetSpeed == 0) {
            if (Math.abs(speed) < coastToStopTol) {
                torqueFrac = 0;
                motor.setPower(0);
                return;
            }
        }

        int dir = targetSpeed < 0 ? -1 : 1;

        if (Math.abs(speed - targetSpeed) < speedTol) torqueFrac = dir * cruiseTorqueFrac;
        else if (speed < targetSpeed) torqueFrac = accelTorqueFrac;
        else torqueFrac = -accelTorqueFrac;

        double deltaTime = currTime - prevTime;
        double minTf = prevTorqueFrac - torqueRamp * deltaTime;
        double maxTf = prevTorqueFrac + torqueRamp * deltaTime;

        if (torqueFrac < minTf) torqueFrac = minTf;
        if (torqueFrac > maxTf) torqueFrac = maxTf;

        double volt = (torqueFrac + speed / motorConf.topSpeed) * motorConf.nominalVolt;
        motor.setPower(volt / vs.getVoltage());
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("tgt spd=%.2f, spd=%.2f, trq=%.2f, pwr=%.2f", targetSpeed, getSpeed(), torqueFrac, motor.getPower());
    }
}
