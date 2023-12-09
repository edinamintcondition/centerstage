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

    //for 20:1
    private static final MoveCal driveCal = new MoveCal(0, 1185, -610, 300);
    private static final MoveCal strafeCal = new MoveCal(0, 418, 0, 0);
    private static final double[] accelTorqueFrac = new double[]{0.9619 * 2.0, 0.9748 * 2.0, 1.0273 * 2.0, 1.0405 * 2.0};
    private static final double[] cruiseTorqueFrac = new double[]{1.24, 1.24, 1.24, 1.24};
    private static final double[] deccelTorqueFrac = new double[]{-2.0, -2.0, -2.0, -2.0};

    //for 40:1
//    public static final double fwdAccel = 418;
//    public static final double fwdDeccel = -29999999;
//    private static final double[] accelTorqueFrac = new double[]{0.9619 * 2.0, 0.9748 * 2.0, 1.0273 * 2.0, 1.0405 * 2.0};
//    private static final double[] cruiseTorqueFrac = new double[]{1.24, 1.24, 1.24, 1.24};
//    private static final double[] deccelTorqueFrac = new double[]{-2.0, -2.0, -2.0, -2.0};

    public TractionControl(HardwareMap hardwareMap, VoltageSensor vs) {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");

        DcMotor[] motors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};

        mCon = new MotorControl2[4];
        for (int i = 0; i < 4; i++) {
            if (i < 2) motors[i].setDirection(FORWARD);
            else motors[i].setDirection(REVERSE);

            mCon[i] = new MotorControl2(motors[i], MotorConfig.driveMotor, vs, accelTorqueFrac[i] / 12, cruiseTorqueFrac[i] / 12, deccelTorqueFrac[i] / 12);
        }
    }

    public MotorControl2 get(int i) {
        return mCon[i];
    }

    public DcMotor[] getMotors() {
        DcMotor[] motors = new DcMotor[4];
        for (int i = 0; i < 4; i++)
            motors[i] = get(i).getMotor();
        return motors;
    }

    public double getDriveSpeed() {
        return getSpeed(false);
    }

    public double getSpeed(boolean strafe) {
        int s = 1;
        if (strafe) s = -1;

        double[] d = new double[]{get(0).getSpeed(), s * get(1).getSpeed(), s * get(2).getSpeed(), get(3).getSpeed()};

        Arrays.sort(d);
        return (d[1] + d[2]) / 2;
    }

    public boolean isStopped() {
        for (int i = 0; i < 4; i++)
            if (!mCon[i].isStopped()) return false;

        return true;
    }

    public void setTargetDriveSpeed(double t) {
        setTargetSpeed(t, false);
    }

    public void setTargetSpeed(double t, boolean strafe) {
        int s = 1;
        if (strafe) s = -1;

        get(0).setTargetSpeed(t);
        get(1).setTargetSpeed(s * t);
        get(2).setTargetSpeed(s * t);
        get(3).setTargetSpeed(t);
    }

    public void resetDeg() {
        for (int i = 0; i < 4; i++)
            get(i).resetDeg();
    }

    public double getDeg(boolean strafe) {
        int s = 1;
        if (strafe) s = -1;

        double[] d = new double[]{get(0).getDeg(), s * get(1).getDeg(), s * get(2).getDeg(), get(3).getDeg()};

        Arrays.sort(d);
        return (d[1] + d[2]) / 2;
    }

    public void run(boolean strafe) {
        for (int i = 0; i < 4; i++)
            get(i).sample();

        double v = getSpeed(strafe);

        int s = 1;
        if (strafe) s = -1;

        get(0).run(v);
        get(1).run(s * v);
        get(2).run(s * v);
        get(3).run(v);
    }

    public void runDistance(double targetDist, boolean strafe) {
        MoveCal mc;
        if (strafe) mc = strafeCal;
        else mc = driveCal;
    }
}
