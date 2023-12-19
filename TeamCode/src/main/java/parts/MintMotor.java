package parts;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Speedometer;

@SuppressLint("DefaultLocale")
public class MintMotor {
    private final DcMotor motor;
    private final VoltageSensor vs;
    private final MotorConfig motorConf;
    private final double accelTorqueFrac;
    private final double cruiseTorqueFrac;
    private final Speedometer speedo;
    private double targetSpeed, torqueFrac, initPos;
    private static final double speedTol = 30;
    private static final double coastToStopTol = 90;

    public MintMotor(DcMotor motor, MotorConfig motorConf, VoltageSensor vs, double accelTf, double coastTf) {
        this.motor = motor;
        this.vs = vs;
        this.motorConf = motorConf;
        this.accelTorqueFrac = accelTf;
        this.cruiseTorqueFrac = coastTf;
        speedo = new Speedometer(8);
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
    }

    public void run(double speed) {
        // if stopping, moving (combine last two)
        if (targetSpeed == 0) {
            if (Math.abs(speed) < coastToStopTol) {
                torqueFrac = 0;
                motor.setPower(0);
                return;
            }
        }

        if (Math.abs(speed - targetSpeed) < speedTol) torqueFrac = cruiseTorqueFrac;
        else if (speed < targetSpeed) torqueFrac = accelTorqueFrac;
        else torqueFrac = -accelTorqueFrac;

        double volt = (torqueFrac + speed / motorConf.topSpeed) * motorConf.nominalVolt;
        motor.setPower(volt / vs.getVoltage());
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("tgt spd=%.2f, spd=%.2f, trq=%.2f, pwr=%.2f", targetSpeed, getSpeed(), torqueFrac, motor.getPower());
    }
}
