package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import parts.MintWheels;

import java.util.List;

@Autonomous
public class NavigationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
//        Positioning pos = new Positioning();
//        MintWheels autoDrive = new MintWheels(hardwareMap, gamepad1, telemetry);
//
//        //set camera
//        WebcamName camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
//        VisionPortal myVisionPort = VisionPortal.easyCreateWithDefaults(camera1, pos.myAprilTagProc);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            Position currentPos = pos.getPosition();
//            if (currentPos == null)
//                continue;
//
//            double rx = currentPos.x;
//            double ry = currentPos.y;
//            double a = currentPos.a;
//            double dx = currentPos.dx;
//            double dy = currentPos.dy;
//
//            double goalY = 120;
//            double goalX = 90;
//            double goalYaw = 0;
//
//            double ty = goalY - ry;
//            double tx = goalX - rx;
//
//            double axial = (tx * dx) + (ty * dy);
//            double lateral = (tx * dy) - (ty * dx);
//            double yaw = a - goalYaw;
//            yaw = 0;
//
//            if (yaw < -180) {
//                yaw = yaw + 360;
//            }
//
//            if (yaw > 180) {
//                yaw = yaw - 360;
//            }
//
//            double powerLimit = 0.25;
//
//            telemetry.addData("r", "%.1f, %.1f", rx, ry);
//            telemetry.addData("a", "%.1f", a);
//            telemetry.addData("d", "%.1f, %.1f", dx,dy);
//            telemetry.addData("ax lat", "%.1f, %.1f", axial, lateral);
//            telemetry.addData("yaw","%.1f",yaw);
//            telemetry.update();
//
//            autoDrive.runAny(axial, lateral, yaw, powerLimit);
 //       }
    }
}