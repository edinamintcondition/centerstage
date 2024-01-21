package parts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;

@SuppressLint("DefaultLocale")
public class MintDrive {
    private final MintMotor[] mCon;

    //for 20:1
    private static final double shutdownTol = 30;
    private static final MoveCal driveCal = new MoveCal(15, 900, -930, 300);
    private static final MoveCal strafeCal = new MoveCal(15, 571, -1200, 300);
    private static final double[] accelVolts = new double[]{0.9619 * 0.25, 0.9748 * 0.25, 1.0273 * 0.25, 1.0405 * 0.25};

    private double[] move;
    private MoveCal amc;
    private double dir, tgtDeg;

    public MintDrive(HardwareMap hardwareMap) {
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

            mCon[i] = new MintMotor(motors[i], MotorConfig.driveMotor, vs, accelTf);
        }
    }

    public MintMotor get(int i) {
        return mCon[i];
    }

    public void resetPos() { // prepMove, reset deg, read imu
        for (int i = 0; i < 4; i++)
            get(i).resetDeg();
    }

    public double getSpeed() {
        double[] d = new double[4];
        for (int i = 0; i < 4; i++)
            d[i] = get(i).getSpeed() * move[i]; // mult/divide?

        Arrays.sort(d);
        return (d[1] + d[2]) / 2;
    }

    public double getDeg() {
        double[] d = new double[4];
        for (int i = 0; i < 4; i++)
            d[i] = get(i).getDeg() * move[i]; // mult/divide?

        Arrays.sort(d);
        return (d[1] + d[2]) / 2;
    }

    public double getPos() {
        double deg = getDeg();
        return deg / amc.dpi;
    }

    public String moveString() {
        return String.format("tgtDeg: %f, dir: %f, move: %f, %f, %f, %f",
                tgtDeg, dir, move[0], move[1], move[2], move[3]);
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
            setDriving(false);

            if (s * dir <= shutdownTol) {
                shutdown();
                return true;
            }
        }

        double cutoff = tgtDeg + 0.5 * dir * s * s / amc.deccel;
        if (d * dir >= cutoff * dir)
            setDriving(false);
        else
            setDriving(true);

        for (int i = 0; i < 4; i++)
            get(i).run(s, move[i] * dir);

        return false;
    }

    private void setDriving(boolean t) {
        for (int i = 0; i < 4; i++)
            get(i).setDriving(t);
    }

    private void shutdown() {
        for (int i = 0; i < 4; i++)
            get(i).shutdown();
    }

    private static VoltageSensor getVs(HardwareMap hardwareMap) {
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            return vs;
        }

        throw new RuntimeException("no voltage sensor");
    }
}
