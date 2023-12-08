package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import parts.MintGrabber;

public abstract class MintAutonomous extends LinearOpMode {

    double gearRatio = 20;
    boolean useAprilTags;

    Positioning posn;

    protected Position currentPos;

    DcMotorEx[] motors, revMotors, fwdMotors;

    DcMotorEx armMotor;

    Servo wristServo;

    Servo myServoL;

    Servo myServoR;

    protected final static double frontStartX = 4, backStartX = 8.5, frontCentralX = frontStartX + 50, backboardX = 36;
    protected final static double frontStartY = 32, backStartY = 84, approachY = 112;

    public MintAutonomous(Position initPos) {
        currentPos = initPos;
    }

    @Override
    public void runOpMode() {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        BNO055IMUNew IMU = hardwareMap.get(BNO055IMUNew.class, "imu");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        myServoL = hardwareMap.get(Servo.class, "grab_servo_L");
        myServoR = hardwareMap.get(Servo.class, "grab_servo_R");
        //MintGrabber.init(grabServo);

        posn = new Positioning(IMU, telemetry);

        /*
        WebcamName camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal myVisionPort = VisionPortal.easyCreateWithDefaults(camera1, posn.myAprilTagProc);
        */

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack};

        fwdMotors = new DcMotorEx[]{leftFront, rightBack};

        revMotors = new DcMotorEx[]{leftBack, rightFront};

        for (DcMotorEx m : motors) {
            PIDFCoefficients c = m.getPIDFCoefficients(RUN_TO_POSITION);
            m.setPIDFCoefficients(RUN_TO_POSITION, c);
            m.setMode(STOP_AND_RESET_ENCODER);
        }

        armMotor.setMode(STOP_AND_RESET_ENCODER);

        waitForStart();

        posn.resetPosition();
        //grabServo.setPosition(MintGrabber.CLOSED_POSITION);

        driveToBackboard();
        pause();

        Position p = posn.getRelPosition(currentPos);
        if (useAprilTags && p != null) {
            telemetry.addData("see april tag", "to drop pixel");
            telemetry.update();

            sleep(500);

            Point tgt = new Point(p.x, p.y - 9);
            strafeToClosestPoint(tgt);
            pause();
            driveToClosestPoint(tgt);
        } else {
            Point tgt = new Point(currentPos.x, currentPos.y + 11);
            driveToClosestPoint(tgt);
        }
        pause();

        dropPixel();
        retractArm();
        pause();
        park();
    }

    public abstract void driveToBackboard();

    public abstract void park();

    public void dropPixel() {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo.setPosition(0.7);

        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive()) {
            if (t.milliseconds() > 4500) {
                myServoL.setPosition(MintGrabber.CLOSED_POSITION_L);
                myServoR.setPosition(MintGrabber.CLOSED_POSITION_R);
            }

            if (t.milliseconds() > 5000) {
                break;
            }

            armMotor.setTargetPosition(-400);
            armMotor.setPower(.5);
            armMotor.setMode(RUN_TO_POSITION);
            sleep(1);
        }
    }

    public void retractArm() {
        wristServo.setPosition(0.1);
        myServoL.setPosition(MintGrabber.OPEN_POSITION_L);
        myServoR.setPosition(MintGrabber.OPEN_POSITION_R);

        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive()) {
            if (t.milliseconds() < 2000) {
                armMotor.setTargetPosition(-400);
                armMotor.setPower(.5);
                armMotor.setMode(RUN_TO_POSITION);
                sleep(1);
            } else if (t.milliseconds() > 3000) {
                break;
            } else {
                armMotor.setTargetPosition(-100);
                armMotor.setPower(.5);
                armMotor.setMode(RUN_TO_POSITION);
                sleep(1);
            }
        }
    }

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
        double ppi = 56;
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