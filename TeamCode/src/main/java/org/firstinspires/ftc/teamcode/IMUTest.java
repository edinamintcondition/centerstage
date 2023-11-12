package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class IMUTest extends LinearOpMode {

    BNO055IMUNew IMU;

    @Override
    public void runOpMode() {
        IMU = hardwareMap.get(BNO055IMUNew.class, "IMU");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        IMU.initialize(myIMUparameters);

        waitForStart();

        while (opModeIsActive()) {
            Orientation myRobotOrientation;
            AngularVelocity myRobotAngularVelocity;

            myRobotAngularVelocity = IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            myRobotOrientation = IMU.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            );

            double xRotationRate = myRobotAngularVelocity.xRotationRate;
            double yRotationRate = myRobotAngularVelocity.yRotationRate;
            double zRotationRate = myRobotAngularVelocity.zRotationRate;
            double xAxis = myRobotOrientation.firstAngle;
            double yAxis = myRobotOrientation.secondAngle;
            double zAxis = myRobotOrientation.thirdAngle;

            telemetry.addData("z rotation", zRotationRate);
            telemetry.addData("x rotation", xRotationRate);
            telemetry.addData("y rotation", yRotationRate);
            telemetry.addData("x angle", xAxis);
            telemetry.addData("z angle", zAxis);
            telemetry.addData("y angle", yAxis);
            telemetry.update();
        }
    }
}