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
    private static final double coastToStopTol = 30;
    private final ElapsedTime t;
    private boolean driving;

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

    public void setDriving(boolean t) {
        driving = t;
    }

    public void resetDeg() {
        speedo.clearSamples();
        initPos = motor.getCurrentPosition();
    }

    public void sample() {
        speedo.sample(getDeg());
    }

    public void run(double currBotSpeed, double dir) {
        if (!driving) {
            if (currBotSpeed < coastToStopTol) {
                torqueFrac = 0;
                motor.setPower(0);
                return;
            } else {
                torqueFrac = -accelTorqueFrac;
            }
        } else {
            torqueFrac = accelTorqueFrac;
        }

        torqueFrac *= dir;
        currBotSpeed *= dir;

        double volt = (torqueFrac + currBotSpeed / motorConf.topSpeed) * motorConf.nominalVolt;
        motor.setPower(volt / vs.getVoltage());
    }

    public void shutdown() {
        motor.setPower(0);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("spd=%.2f, trq=%.2f, pwr=%.2f",
                getSpeed(), torqueFrac, motor.getPower());
    }
}
