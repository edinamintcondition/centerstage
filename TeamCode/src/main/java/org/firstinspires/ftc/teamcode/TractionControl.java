package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class TractionControl {
    private final MotorControl2[] mCon;

    private final double coastSpeed = 400;

    private static final double[] accelTorqueFrac = new double[]{
            2.0 / 12,
            2.0 / 12,
            2.0 / 12,
            2.0 / 12
    };

    private static final double[] cruiseTorqueFrac = new double[]{
            1.05 / 12,
            1.05 / 12,
            1.05 / 12,
            1.05 / 12
    };

    private static final double[] deccelTorqueFrac = new double[]{
            -accelTorqueFrac[0],
            -accelTorqueFrac[1],
            -accelTorqueFrac[2],
            -accelTorqueFrac[3]
    };

    public TractionControl(DcMotor[] motors, VoltageSensor vs) {
        mCon = new MotorControl2[4];
        for (int i = 0; i < 4; i++)
            mCon[i] = new MotorControl2(motors[i], MotorConfig.driveMotor, vs,
                    accelTorqueFrac[i], cruiseTorqueFrac[i], deccelTorqueFrac[i]);
    }

    public MotorControl2 get(int i) {
        return mCon[i];
    }

    public double getDriveSpeed() {
        double s = 0;
        for (int i = 0; i < 4; i++)
            s += mCon[i].getSpeed();
        return s / 4;
    }

    public void setTargetDriveSpeed(double t) {
        for (int i = 0; i < 4; i++)
            mCon[i].setTargetSpeed(t);
    }

    public void run() {
        for (int i = 0; i < 4; i++)
            mCon[i].run();
    }
}
