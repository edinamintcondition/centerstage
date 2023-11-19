package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class AutoOpMode extends LinearOpMode {

    double gearRatio = 20;

    Positioning posn;

    Position currentPos;

    DcMotorEx[] motors, revMotors, fwdMotors;

    public AutoOpMode(Position initPos) {
        currentPos = initPos;
    }

    @Override
    public void runOpMode() {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        BNO055IMUNew IMU = hardwareMap.get(BNO055IMUNew.class, "imu");

        posn = new Positioning(IMU, telemetry);

//        WebcamName camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
//        VisionPortal myVisionPort = VisionPortal.easyCreateWithDefaults(camera1, posn.myAprilTagProc);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotorEx[]{
                leftFront,
                leftBack,
                rightFront,
                rightBack
        };

        fwdMotors = new DcMotorEx[]{
                leftFront,
                rightBack
        };

        revMotors = new DcMotorEx[]{
                leftBack,
                rightFront
        };

        for (DcMotorEx m : motors) {
            PIDFCoefficients c = m.getPIDFCoefficients(RUN_TO_POSITION);
            m.setPIDFCoefficients(RUN_TO_POSITION, c);
            m.setMode(STOP_AND_RESET_ENCODER);
        }

        waitForStart();

        driveToBackboard();
    }

    public abstract void driveToBackboard();

    public void pause() {
        telemetry.addData("pos", "%f %f", currentPos.x, currentPos.y);
        telemetry.update();

        sleep(100);
    }

    public void driveToClosestPoint(Point target) {
        double ppi = 537.0 * 5 / 55.9;
        ppi = ppi * (gearRatio / 20);
        double fastLimit = 10;

        Position updatedPos = null;

        while (opModeIsActive()) {
            Position newCurPos = posn.getPosition();
            if (newCurPos != null) {
                this.currentPos = newCurPos;
                updatedPos = null;
            }

            if (updatedPos == null) {
                double targetDist = currentPos.toRobotRel(target).y;
                if (Math.abs(targetDist) < 0.1) {
                    break;
                }

                if (newCurPos != null)
                    telemetry.addData("april tag pos", "%f %f", newCurPos.x, newCurPos.y);
                telemetry.addData("drive", targetDist);
                pause();

                updatedPos = currentPos.addRobotRel(new Point(0, targetDist));

                int targetPos = (int) (targetDist * ppi);
                double power = 0.5;

                for (DcMotor m : motors) {
                    int p = m.getCurrentPosition();
                    m.setTargetPosition(p + targetPos);
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

    public void strafeToClosestPoint(Point target) {
        double fastLimit = 10;
        double ppi = 58;
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

                if (newCurPos != null)
                    telemetry.addData("april tag pos", "%f %f", newCurPos.x, newCurPos.y);
                telemetry.addData("strafe", targetDist);
                pause();

                updatedPos = currentPos.addRobotRel(new Point(targetDist, 0));

                int targetPos = (int) (targetDist * ppi);
                double power = 0.5;

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

    public void rotateToHeading(double targetHeading) {
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

                telemetry.addData("rotate", targetAngle);
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

    public boolean areIdle() {
        for (DcMotor m : motors) {
            if (m.isBusy()) {
                return false;
            }
        }
        return true;
    }
}