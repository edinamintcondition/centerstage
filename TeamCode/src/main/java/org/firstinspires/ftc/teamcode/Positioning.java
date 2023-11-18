package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

public class Positioning {
    public AprilTagProcessor myAprilTagProc;

    private BNO055IMUNew IMU;

    private double intialHeading;
    private final double[] allmx = {29.381, 35.381, 41.381, 100.0435, 106.0435, 112.0435};
    private final double[] allmy = {132.492908, 132.492908, 132.492908, 132.492908, 132.492908, 132.492908, 0, 0, 0, 0};

    public Positioning(BNO055IMUNew IMU) {
        this.IMU = IMU;

        AprilTagProcessor.Builder myAprilTagProcBuilder = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true);

        myAprilTagProc = myAprilTagProcBuilder.build();

        com.qualcomm.robotcore.hardware.IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        IMU.initialize(myIMUparameters);

        intialHeading = getHeading();
    }

    public Position getPosition() { //rename
        List<AprilTagDetection> currentDetections = myAprilTagProc.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id <= 6) {
                int i = detection.id - 1;

                double mx = allmx[i];
                double my = allmy[i];

                double mc = Math.sqrt((detection.ftcPose.x * detection.ftcPose.x) + (detection.ftcPose.y * detection.ftcPose.y));

                double a = getHeading();
                double dx = Math.sin(Math.toRadians(detection.ftcPose.yaw));
                double dy = Math.cos(Math.toRadians(detection.ftcPose.yaw));

                double qc = dx * mc;
                double rx = qc + mx;

                double mq = dy * mc;
                double ry = my - mq;

                return new Position(rx, ry, dx, dy, a);
            }
        }

        return null;
    }

    public double getHeading() {
        Orientation myRobotOrientation;

        myRobotOrientation = IMU.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        );

        double zAxis = myRobotOrientation.thirdAngle;

        return zAxis - intialHeading;
    }


    public void reset() {
        intialHeading = getHeading();
    }

}