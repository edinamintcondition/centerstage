package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@SuppressLint("DefaultLocale")
public class MotorCalMode extends LinearOpMode {
    private VoltageSensor vs;
    private final MotorConfig mConf = MotorConfig.driveMotor;

    @Override
    public void runOpMode() {
        getVs();

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] motors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};

        waitForStart();

        telemetry.addData("test", "motor starting voltage");
        telemetry.update();

        double startVolts = testStartVolts(motors);

        telemetry.addData("test", "motor acceleration");
        telemetry.update();

        double[] accel = testAccel(motors, 0.66 * startVolts / 12);
        dump(motors, startVolts, accel);

        double[] accel2 = testAccel(motors, -0.66 * startVolts / 12, accel);
        dump(motors, startVolts, accel2);

        while (opModeIsActive()) {
            sleep(1);
        }
    }

    private double testStartVolts(DcMotor[] m) {
        double[] initPos = new double[4];
        for (int i = 0; i < 4; i++) {
            m[i].setMode(RUN_WITHOUT_ENCODER);
            m[i].setZeroPowerBehavior(BRAKE);
            m[i].setPower(0);

            initPos[i] = m[i].getCurrentPosition();
        }

        double volt = 0;
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            int moved = 0;
            for (int i = 0; i < 4; i++) {
                if (Math.abs(m[i].getCurrentPosition() - initPos[i]) > 4)
                    moved++;
            }

            if (moved == 4)
                break;

            volt = t.seconds() / 2;
            if (volt > 12) throw new RuntimeException("motor did not start");

            telemetry.addData("volt", String.format("%.02f", volt));
            telemetry.update();

            for (int i = 0; i < 4; i++)
                m[i].setPower(volt / vs.getVoltage());
        }

        for (int i = 0; i < 4; i++)
            m[i].setPower(0);
        sleep(50);

        return volt;
    }

    private double[] testAccel(DcMotor[] m, double torqueFrac) {
        double[] scale = new double[4];
        for (int i = 0; i < 4; i++)
            scale[i] = 1;

        return testAccel(m, torqueFrac, scale);
    }

    private double[] testAccel(DcMotor[] m, double torqueFrac, double[] scale) {
//        double avg = 0;
//        for (int i = 0; i < 4; i++)
//            avg += scale[i];
//        avg /= m.length;

        double[] tf = new double[4];
        for (int i = 0; i < 4; i++)
            tf[i] = torqueFrac;

        if (Math.abs(torqueFrac) > 0.75)
            throw new RuntimeException("torque frac should be <= 0.75");

        Speedometer[] s = new Speedometer[4];
        Accelerometer[] a = new Accelerometer[4];
        for (int i = 0; i < 4; i++) {
            s[i] = new Speedometer(4);
            a[i] = new Accelerometer(10000);
        }

        boolean done = false;
        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                double motorCurrDeg = mConf.toDeg(m[i].getCurrentPosition());
                s[i].sample(motorCurrDeg);
                a[i].sample(motorCurrDeg);

                double currSpeed = s[i].getSpeed();
                if (Math.abs(currSpeed) > 0.75 * mConf.topSpeed) {
                    done = true;
                    break;
                }

                double volt = (tf[i] + currSpeed / mConf.topSpeed) * mConf.nominalVolt;
                m[i].setPower(volt / vs.getVoltage());
            }

            if (done) break;
        }

        double[] totalAccel = new double[m.length];
        for (int i = 0; i < 4; i++) {
            totalAccel[i] = a[i].getAccel();
            m[i].setZeroPowerBehavior(BRAKE);
            m[i].setPower(0);
        }

        return totalAccel;
    }

    private void dump(DcMotor[] motors, double startVolts, double[] accel) {
        telemetry.addData("v start", String.format("%.2f", startVolts));
        for (int i = 0; i < 4; i++) {
            double pos = mConf.toPos(motors[i].getCurrentPosition());
            String s = String.format("p=%.2f, a=%.2f", pos, accel[i]);
            telemetry.addData("motor" + i, s);
        }
        telemetry.update();
    }

    private void getVs() {
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            this.vs = vs;
            return;
        }

        throw new RuntimeException("no voltage sensor");
    }
}
