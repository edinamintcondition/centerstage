package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@SuppressLint("DefaultLocale")
public class MotorControl2 {
    private final DcMotor motor;
    private final VoltageSensor vs;
    private final MotorConfig motorConf;
    private final double accelTorqueFrac, cruiseTorqueFrac, deccelTorqueFrac;
    private final Speedometer speedo;
    private double targetSpeed, torqueFrac, initPos;
    private static final double speedTol = 10;
    private static final double coastToStopTol = 90;

    public MotorControl2(DcMotor motor, MotorConfig motorConf, VoltageSensor vs, double accelTf, double coastTf) {
        this(motor, motorConf, vs, accelTf, coastTf, accelTf - coastTf);
    }

    public MotorControl2(DcMotor motor, MotorConfig motorConf, VoltageSensor vs, double accelTf, double coastTf, double deccelTf) {
        this.motor = motor;
        this.vs = vs;
        this.motorConf = motorConf;
        this.accelTorqueFrac = accelTf;
        this.cruiseTorqueFrac = coastTf;
        this.deccelTorqueFrac = deccelTf;
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
        if (targetSpeed == 0) {
            if (Math.abs(speed) < coastToStopTol) torqueFrac = 0;
            else if (speed > 0) torqueFrac = deccelTorqueFrac;
            else torqueFrac = -deccelTorqueFrac;
        } else if (targetSpeed > 0) {
            if (Math.abs(speed - targetSpeed) < speedTol) torqueFrac = cruiseTorqueFrac;
            else if (speed < targetSpeed) torqueFrac = accelTorqueFrac;
            else torqueFrac = deccelTorqueFrac;
        } else {
            if (Math.abs(speed - targetSpeed) < speedTol) torqueFrac = -cruiseTorqueFrac;
            else if (speed < targetSpeed) torqueFrac = -deccelTorqueFrac;
            else torqueFrac = -accelTorqueFrac;
        }

        if (torqueFrac == 0) {
            motor.setPower(0);
        } else {
            double volt = (torqueFrac + speed / motorConf.topSpeed) * motorConf.nominalVolt;
            motor.setPower(volt / vs.getVoltage());
        }
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("tgt spd=%.2f, spd=%.2f, trq=%.2f, pwr=%.2f", targetSpeed, getSpeed(), torqueFrac, motor.getPower());
    }
}