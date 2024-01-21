package parts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class MintDrive {
    private final MintMotor[] mCon;

    // use calibration multipliers in different array

    //for 20:1
    private static final MoveCal driveCal = new MoveCal(15, 900, -930, 300);
    private static final MoveCal strafeCal = new MoveCal(15, 571, -1200, 300);
    private static final double[] accelVolts = new double[]{0.9619 * 0.25, 0.9748 * 0.25, 1.0273 * 0.25, 1.0405 * 0.25};
    private static final double[] cruiseVolts = new double[]{0.9619 * 0.18, 0.9748 * 0.18, 1.0273 * 0.18, 1.0405 * 0.18};
    private static final double maxSpeed = 300;
    private static final double torqueRampTime = 0.2;
    private Telemetry telemetry;

    //for 40:1
//    public static final double fwdAccel = 418;
//    public static final double fwdDeccel = -29999999;
//    private static final double[] accelTorqueFrac = new double[]{0.9619 * 2.0, 0.9748 * 2.0, 1.0273 * 2.0, 1.0405 * 2.0};
//    private static final double[] cruiseTorqueFrac = new double[]{1.24, 1.24, 1.24, 1.24};
//    private static final double[] deccelTorqueFrac = new double[]{-2.0, -2.0, -2.0, -2.0};

    public MintDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");

        DcMotor[] motors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};

        VoltageSensor vs = getVs(hardwareMap);

        mCon = new MintMotor[4];
        for (int i = 0; i < 4; i++) {
            if (i < 2) motors[i].setDirection(FORWARD);
            else motors[i].setDirection(REVERSE);

            double accelTf = accelVolts[i];
            double coastTf = cruiseVolts[i];
            double torqueRamp = accelTf / torqueRampTime;

            mCon[i] = new MintMotor(motors[i], MotorConfig.driveMotor, vs, accelTf, coastTf, torqueRamp);
        }
    }

    public MintMotor get(int i) {
        return mCon[i];
    }

//    public double getSpeed(boolean strafe) {
//        int s = 1;
//        if (strafe) s = -1;
//
//        double[] d = new double[]{get(0).getSpeed(), s * get(1).getSpeed(), s * get(2).getSpeed(), get(3).getSpeed()};
//
//        Arrays.sort(d);
//        return (d[1] + d[2]) / 2;
//    }

//    public void setTargetSpeed(double t, boolean strafe) {
//        int s = 1;
//        if (strafe) s = -1;
//
//        get(0).setTargetSpeed(t);
//        get(1).setTargetSpeed(s * t);
//        get(2).setTargetSpeed(s * t);
//        get(3).setTargetSpeed(t);
//    }

    public void resetPos() { // prepMove, reset deg, read imu
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

//    public double getPos(boolean strafe) {
//        double deg = getDeg(strafe);
//        return deg / aDP.getDpi();
//    }
//
//    public void run(boolean strafe) {
//        for (int i = 0; i < 4; i++)
//            get(i).sample();
//
//        double v = getSpeed(strafe);
//
//        int s = 1;
//        if (strafe) s = -1;
//
//        get(0).run(v);
//        get(1).run(s * v);
//        get(2).run(s * v);
//        get(3).run(v);
//    }
//
//    private double tAccel;
//    private double tCruise;
//    private double tDeccel;
//    private double targetDegs;
//    private double targetDir;
//    private DynamicParams aDP;
//    private double degStopAccel, degStopDeccel;
//
//    public DynamicParams getActiveDynamicParams() {
//        return aDP;
//    }

//    private boolean runTo(double targetPos, boolean strafe, int dir) {
//        MoveCal mc;
//        if (strafe) mc = strafeCal;
//        else mc = driveCal;
//
//        double s = getSpeed(strafe);
//        double d = getDeg(strafe);
//        double tgtDeg = mc.dpi * targetPos;
//
//        if (d * dir >= tgtDeg * dir) {
//            setTargetSpeed(0, strafe);
//            return true;
//        }
//
//        double tgtSpeed;
//        double cutoff = tgtDeg + 0.5 * dir * s * s / mc.deccel;
//        if (d * dir >= cutoff * dir) tgtSpeed = 0;
//        else tgtSpeed = dir * mc.maxSpeed;
//
//        for (int i = 0; i < 4; i++)
//            telemetry.addData("motor" + i, get(i));
//        telemetry.addData("pos", d / mc.dpi);
//        telemetry.addData("tgtSpeed", tgtSpeed);
//        telemetry.update();
//
//        setTargetSpeed(tgtSpeed, strafe);
//
//        run(strafe);
//
//        return false;
//    }

    private double[] move;
    private MoveCal amc;
    private double dir, tgtDeg;

    public double getSpeed() {
        double[] d = new double[4];
        for (int i = 0; i < 4; i++)
            d[i] = get(1).getSpeed() * move[i]; // mult/divide?

        Arrays.sort(d);
        return (d[1] + d[2]) / 2;
    }

    public double getDeg() {
        double[] d = new double[4];
        for (int i = 0; i < 4; i++)
            d[i] = get(1).getDeg() * move[i]; // mult/divide?

        Arrays.sort(d);
        return (d[1] + d[2]) / 2;
    }

    public double getPos() {
        double deg = getDeg();
        return deg / amc.dpi;
    }

    public void preRun(double targetPos, boolean strafe) {
        if (strafe) {
            move = new double[]{1, -1, -1, 1};
            amc = strafeCal;
        } else {
            move = new double[]{1, 1, 1, 1};
            amc = driveCal;
        }

        tgtDeg = amc.dpi * targetPos;
        dir = Math.signum(tgtDeg - getDeg());
    }

    public boolean run() {
        for (int i = 0; i < 4; i++)
            get(i).sample();
        double s = getSpeed();
        double d = getDeg();

        if (d * dir >= tgtDeg * dir) {
            setTargetSpeed(0);
            return true;
        }

        double tgtSpeed;
        double cutoff = tgtDeg + 0.5 * dir * s * s / amc.deccel;
        if (d * dir >= cutoff * dir) tgtSpeed = 0;
        else tgtSpeed = dir * amc.maxSpeed;

        setTargetSpeed(tgtSpeed);

        for (int i = 0; i < 4; i++)
            get(0).run(s * move[i]);

        return false;
    }

    private void setTargetSpeed(double t) {
        for (int i = 0; i < 4; i++)
            get(i).setTargetSpeed(t * move[i]);
    }

    private static VoltageSensor getVs(HardwareMap hardwareMap) {
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            return vs;
        }

        throw new RuntimeException("no voltage sensor");
    }
}
