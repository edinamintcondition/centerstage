package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import data.Point;
import data.Position;
import parts.MintGrabber;
import parts.MintWrist;
import parts.MintDrive;

public abstract class MintAutonomous extends LinearOpMode {

    double gearRatio = 20;
    boolean useAprilTags = true;

    Positioning posn;

    protected Position currentPos;

    DcMotorEx[] motors, revMotors, fwdMotors;

    DcMotorEx armMotor;

    Servo wristServo;

    Servo myServoL;

    Servo myServoR;

    protected final static double frontStartX = 4, backStartX = 8.5, frontCentralX = frontStartX + 58, backboardX = 33;
    protected final static double frontStartY = 32, backStartY = 84, approachY = 112;
    protected final static double parkX = 22;

    public MintAutonomous(Position initPos) {
        currentPos = initPos;
    }

    MintDrive tc;

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
        tc = new MintDrive(hardwareMap, telemetry);
        //MintGrabber.init(grabServo);

        posn = new Positioning(IMU, telemetry);

        WebcamName camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal myVisionPort = VisionPortal.easyCreateWithDefaults(camera1, posn.myAprilTagProc);


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

        myServoL.setPosition(MintGrabber.CLOSED_POSITION_L);
        myServoL.setPosition(MintGrabber.OPEN_POSITION_R);
        wristServo.setPosition(MintWrist.FLAT_POSITION);
        driveToBackboard();
        pause();

        liftArm();
        pause();

        Position p = posn.getRelPosition(currentPos);
        if (useAprilTags && p != null) {
            telemetry.addData("see april tag", "to drop pixel");
            telemetry.update();

            sleep(200);

            Point tgt = new Point(p.x, p.y - 8);
            strafeToClosestPoint(tgt);
            pause();
            driveToClosestPoint(tgt);
        } else {
            Point tgt = new Point(currentPos.x, currentPos.y + 11);
            driveToClosestPoint(tgt);
        }
        pause();

        retractArm();
        dropPixel();
        park();
    }

    public abstract void driveToBackboard();

    public abstract void park();

    public void liftArm() {
        wristServo.setPosition(MintWrist.DROP_POSITION);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive()) {
            if (t.milliseconds() > 2000) {
                break;
            }

            armMotor.setTargetPosition(-500);
            armMotor.setPower(.5);
            armMotor.setMode(RUN_TO_POSITION);
            sleep(1);
        }
    }

    public void dropPixel() {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime t = new ElapsedTime();
        t.reset();

        double delay = 500;
        while (opModeIsActive()) {
            if (t.milliseconds() > delay) {
                myServoL.setPosition(MintGrabber.OPEN_POSITION_L);
                myServoR.setPosition(MintGrabber.OPEN_POSITION_R);
            }

            if (t.milliseconds() > delay + 500) {
                break;
            }

            armMotor.setTargetPosition(-500);
            armMotor.setPower(.5);
            armMotor.setMode(RUN_TO_POSITION);
            sleep(1);
        }
    }

    public void retractArm() {
        wristServo.setPosition(MintWrist.RETRACTED_POS);
        myServoL.setPosition(MintGrabber.CLOSED_POSITION_L);
        myServoR.setPosition(MintGrabber.CLOSED_POSITION_R);

//        ElapsedTime t = new ElapsedTime();
//        t.reset();
//
//        while (opModeIsActive()) {
//            if (t.milliseconds() < 2000) {
//                armMotor.setTargetPosition(-400);
//                armMotor.setPower(.5);
//                armMotor.setMode(RUN_TO_POSITION);
//                sleep(1);
//            } else if (t.milliseconds() > 3000) {
//                break;
//            } else {
//                armMotor.setTargetPosition(-100);
//                armMotor.setPower(.5);
//                armMotor.setMode(RUN_TO_POSITION);
//                sleep(1);
//            }
//        }
    }

    public void pause() {
        telemetry.addData("pos", "%f %f", currentPos.x, currentPos.y);
        telemetry.update();

        sleep(250);
    }

    public void driveToClosestPoint(Point target) {
        preMove();

        Position initPos = currentPos;
        double tgtDist = currentPos.toRobotRel(target).y;

        if (Math.abs(tgtDist) < 0.1) return;

        telemetry.addData("driving", tgtDist);
        pause();

        tc.resetDeg();

        while (opModeIsActive()) {
            if (tgtDist > 0) {
                boolean done = tc.runForwardTo(tgtDist, false);
                if (done) break;
            } else {
                boolean done = tc.runBackTo(tgtDist, false);
                if (done) break;
            }
        }

        driveToStop(false);

        this.currentPos = initPos.addRobotRel(new Point(0, tgtDist));

        telemetry.addData("drive done", this.currentPos);
        pause();
    }

    private void driveToStop(boolean strafe) {
        tc.setTargetSpeed(0, strafe);
        while (opModeIsActive()) {
            tc.run(strafe);
            if (tc.isStopped())
                break;
        }
    }

    private void preMove() {
        for (DcMotor m : motors) {
            m.setPower(0);
            m.setMode(RUN_USING_ENCODER);
        }
    }

    public void strafeToClosestPoint(Point target) { // extract method with driveToClosestPoint
        preMove();

        Position initPos = currentPos;
        double tgtDist = currentPos.toRobotRel(target).x;

        if (Math.abs(tgtDist) < 0.1) return;

        tc.resetDeg();

        while (opModeIsActive()) {
            if (tgtDist > 0) {
                boolean done = tc.runForwardTo(tgtDist, true);
                if (done) break;
            } else {
                boolean done = tc.runBackTo(tgtDist, true);
                if (done) break;
            }
        }

        driveToStop(true);

        this.currentPos = initPos.addRobotRel(new Point(tgtDist, 0));

        telemetry.addData("strafe done", this.currentPos);
        pause();
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