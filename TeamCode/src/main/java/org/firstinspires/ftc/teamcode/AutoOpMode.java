package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoOpMode extends LinearOpMode {

    double gearRatio = 40;

    Positioning posn;

    Position currentPos;

    DcMotor[] motors, revMotors, fwdMotors;

    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");
        BNO055IMUNew IMU = hardwareMap.get(BNO055IMUNew.class, "IMU");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotor[]{
                leftFront,
                leftBack,
                rightFront,
                rightBack
        };

        fwdMotors = new DcMotor[]{
                leftFront,
                rightBack
        };

        revMotors = new DcMotor[]{
                leftBack,
                rightFront
        };

        waitForStart();

        posn = new Positioning(IMU);
        currentPos = new Position(12, 36, 0, 1, 0);

        telemetry.addData("heading", posn.getHeading());
        telemetry.update();

        for (DcMotor m : motors) {
            m.setMode(STOP_AND_RESET_ENCODER);
        }

        strafeToClosestPoint(new Point(15, 36));
        sleep(200);
        driveDistance(40);
        sleep(200);
        strafeDistance(16);
        sleep(200);
        driveDistance(5);
        sleep(200);
        driveDistance(-5);
        sleep(200);
        rotateAmount(180);
    }

    private void driveDistance(double targetDist) {
        double ppi = 537.0 * 5 / 60.875;
        ppi = ppi * (gearRatio / 20);
        double fastLimit = 10;

        int targetPos = (int) (targetDist * ppi);
        double power;

        if (targetDist > fastLimit) {
            power = 1;
        } else {
            power = .25;
        }

        for (DcMotor m : motors) {
            int p = m.getCurrentPosition();
            m.setTargetPosition(p + targetPos);
            m.setPower(power);
            m.setMode(RUN_TO_POSITION);
        }

        while (opModeIsActive()) {
            if (areIdle()) {
                break;
            }
        }
    }

    private void strafeToClosestPoint(Point target) {
        double fastLimit = 10;
        double ppi = 50.09;
        ppi = ppi * (gearRatio / 20);

        Position updatedPos = null;

        while (opModeIsActive()) {
            Position newCurPos = posn.getPosition();
            if (newCurPos != null) {
                this.currentPos = newCurPos;
                updatedPos = null;
            }

            if (updatedPos == null) {
                double targetDist = currentPos.toRobotRel(target).x;
                if (Math.abs(targetDist) < 0.1) {
                    break;
                }

                updatedPos = currentPos.addRobotRel(new Point(targetDist, 0));

                int targetPos = (int) (targetDist * ppi);
                double power;

                if (targetDist > fastLimit) {
                    power = 1;
                } else {
                    power = .3;
                }

                for (DcMotor m : fwdMotors) {
                    int p = m.getCurrentPosition();
                    m.setTargetPosition(p + targetPos);
                    m.setPower(power);
                    m.setMode(RUN_TO_POSITION);
                }

                for (DcMotor m : revMotors) {
                    int p = m.getCurrentPosition();
                    m.setTargetPosition(p - targetPos);
                    m.setPower(power);
                    m.setMode(RUN_TO_POSITION);
                }
            }

            if (areIdle()) { // shouldn't really happen
                break;
            }
        }

        while (opModeIsActive()) {
            if (areIdle()) {
                break;
            }
        }

        if (updatedPos != null) {
            this.currentPos = updatedPos;
        }
    }

    private void rotateToHeading(double targetHeading) {
        double ppd = 537.0 / 63.15;
        ppd = ppd * (gearRatio / 20);

        while (opModeIsActive()) {
            double targetAngle = targetHeading - posn.getHeading();

            if (targetAngle < -180) {
                targetAngle = targetAngle + 360;
            }
            if (targetAngle > 180) {
                targetAngle = targetAngle - 360;
            }

            if (Math.abs(targetAngle) > 2) {
                int targetPos = (int) (targetAngle * ppd);
                double power = 0.5;

                telemetry.addData("target pos", targetPos);
                telemetry.update();

                for (DcMotor m : motors) {
                    int p = m.getCurrentPosition();
                    if (m.getDirection() == DcMotorSimple.Direction.FORWARD) {
                        m.setTargetPosition(p - targetPos);
                    } else {
                        m.setTargetPosition(p + targetPos);
                    }

                    m.setPower(power);
                    m.setMode(RUN_TO_POSITION);
                }
            } else if (areIdle()) {
                break;
            }
        }
    }

    private void rotateAmount(double targetAngle) {
        double ppd = 537.0 / 63.15;
        ppd = ppd * (gearRatio / 20);

        int targetPos = (int) (targetAngle * ppd);
        double power = 0.5;

        for (DcMotor m : motors) {
            int p = m.getCurrentPosition();
            if (m.getDirection() == DcMotorSimple.Direction.FORWARD) {
                m.setTargetPosition(p - targetPos);
            } else {
                m.setTargetPosition(p + targetPos);
            }

            m.setPower(power);
            m.setMode(RUN_TO_POSITION);
        }

        while (opModeIsActive()) {
            if (areIdle()) {
                break;
            }
        }
    }

    private void strafeDistance(double targetDist) {
        double fastLimit = 10;
        double ppi = 50.09;
        ppi = ppi * (gearRatio / 20);

        int targetPos = (int) (targetDist * ppi);
        double power;

        if (targetDist > fastLimit) {
            power = 1;
        } else {
            power = .3;
        }

        for (DcMotor m : fwdMotors) {
            int p = m.getCurrentPosition();
            m.setTargetPosition(p + targetPos);
            m.setPower(power);
            m.setMode(RUN_TO_POSITION);
        }

        for (DcMotor m : revMotors) {
            int p = m.getCurrentPosition();
            m.setTargetPosition(p - targetPos);
            m.setPower(power);
            m.setMode(RUN_TO_POSITION);
        }

        while (opModeIsActive()) {
            if (areIdle()) {
                break;
            }
        }
    }

    private boolean areIdle() {
        for (DcMotor m : motors) {
            if (m.isBusy()) {
                return false;
            }
        }
        return true;
    }
}