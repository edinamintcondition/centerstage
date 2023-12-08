package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;

public class TractionControl {
    private final MotorControl2[] mCon;
    private final int[] strafeMult;

    //for 20:1

//    private static final double[] accelTorqueFrac = new double[]{
//            2.0 / 12,
//            2.0 / 12,
//            2.0 / 12,
//            2.0 / 12
//    };
//
//    private static final double[] cruiseTorqueFrac = new double[]{
//            1.05 / 12,
//            1.05 / 12,
//            1.05 / 12,
//            1.05 / 12
//    };

    //for 40:1
    public static final double fwdAccel = 418;
    public static final double fwdDeccel = -29999999;
    private static final double[] accelTorqueFrac = new double[]{0.9619 * 2.0, 0.9748 * 2.0, 1.0273 * 2.0, 1.0405 * 2.0};
    private static final double[] cruiseTorqueFrac = new double[]{1.24, 1.24, 1.24, 1.24};
    private static final double[] deccelTorqueFrac = new double[]{-2.0, -2.0, -2.0, -2.0};

    public TractionControl(HardwareMap hardwareMap, VoltageSensor vs) {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");

        DcMotor[] motors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};
        strafeMult = new int[]{1, -1, -1, 1};

        mCon = new MotorControl2[4];
        for (int i = 0; i < 4; i++) {
            if (i < 2)
                motors[i].setDirection(FORWARD);
            else
                motors[i].setDirection(REVERSE);

            mCon[i] = new MotorControl2(motors[i], MotorConfig.driveMotor, vs, accelTorqueFrac[i] / 12, cruiseTorqueFrac[i] / 12, deccelTorqueFrac[i] / 12);
        }
    }

    public MotorControl2 get(int i) {
        return mCon[i];
    }

    public DcMotor[] getMotors() {
        DcMotor[] motors=new DcMotor[4];
        for(int i=0;i<4;i++)
            motors[i]=get(i).getMotor();
        return motors;
    }

    public double getDriveSpeed() {
        double s = 0;
        for (int i = 0; i < 4; i++)
            s += mCon[i].getSpeed();
        return s / 4;
    }

    public boolean isStopped() {
        for (int i = 0; i < 4; i++)
            if (!mCon[i].isStopped()) return false;

        return true;
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
