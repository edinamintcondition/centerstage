package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class NavigationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        AprilTagPositioning pos = new AprilTagPositioning();

        //set camera
        WebcamName camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal myVisionPort = VisionPortal.easyCreateWithDefaults(camera1, pos.myAprilTagProc);

        //set motors
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        //Sets motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            Position currentPos = pos.getPosition();
            if (currentPos == null)
                continue;

            double rx = currentPos.x;
            double ry = currentPos.y;
            double a = currentPos.a;
            double dx = currentPos.dx;
            double dy = currentPos.dy;

            double goalY = 120;
            double goalX = 98;
            double goalYaw = 0;

            double ty = goalY - ry;
            double tx = goalX - rx;

            double axial = (tx * dx) + (ty * dy);
            double lateral = (tx * dy) - (ty * dx);
            double yaw = a - goalYaw;
            yaw = 0;

            if (yaw < -180) {
                yaw = yaw + 360;
            }

            if (yaw > 180) {
                yaw = yaw - 360;
            }

            //These turn a joystick input into a number
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            //Max power of any motor, either direction
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            double powerLimit = 0.25;
            if (max > powerLimit) {
                leftFrontPower *= powerLimit / max;
                rightFrontPower *= powerLimit / max;
                leftBackPower *= powerLimit / max;
                rightBackPower *= powerLimit / max;
            }

            //Applies the power to the motors
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("rx", rx);
            telemetry.addData("ry", ry);
            telemetry.addData("lateral", lateral);
            telemetry.addData("axial", axial);
            telemetry.addData("yaw", yaw);
            telemetry.update();
        }
    }
}