package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import parts.MintArm;
import parts.MintGrabber;
import parts.MintWrist;

@TeleOp
public class OtherGrabber extends LinearOpMode {

    @Override
    public void runOpMode() {
        MintArm arm = new MintArm(hardwareMap, gamepad2, telemetry);
        MintGrabber grabber = new MintGrabber(hardwareMap, gamepad2, telemetry);

        waitForStart();

        telemetry.addData(">", "Starting Program");
        grabber.printPosition();
        telemetry.update();

        while (opModeIsActive()) {

            arm.run();

            grabber.run();
            grabber.printPosition();

            telemetry.update();

        }
    }
}
