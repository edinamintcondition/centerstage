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
    private final Speedometer speedo;
    private double targetSpeed, initPos;
    private double torqueFrac;
    private double currTime;
    private static final double coastToStopTol = 90;
    private final ElapsedTime t;

    public MintMotor(DcMotor motor, MotorConfig motorConf, VoltageSensor vs, double accelTf) {
        this.motor = motor;
        this.vs = vs;
        this.motorConf = motorConf;
        this.accelTorqueFrac = accelTf;
        speedo = new Speedometer(8);
        t = new ElapsedTime();
    }

    public double getDeg() {
        return motorConf.toDeg(motor.getCurrentPosition() - initPos);
    }

    public double getSpeed() {
        return speedo.getSpeed();
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
        currTime = t.seconds();
    }

    public void run(double speed) {
        if (targetSpeed == 0) {
            if (Math.abs(speed) < coastToStopTol) {
                torqueFrac = 0;
                motor.setPower(0);
                return;
            } else {
                torqueFrac = -accelTorqueFrac;
            }
        } else {
            torqueFrac = accelTorqueFrac;
        }

        double volt = (torqueFrac + speed / motorConf.topSpeed) * motorConf.nominalVolt;
        motor.setPower(volt / vs.getVoltage());
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("tgt spd=%.2f, spd=%.2f, trq=%.2f, pwr=%.2f",
                targetSpeed, getSpeed(), torqueFrac, motor.getPower());
    }
}
