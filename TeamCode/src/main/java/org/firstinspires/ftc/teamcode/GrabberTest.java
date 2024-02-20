package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import parts.MintGrabber;
import parts.MintWrist;

@TeleOp
@Disabled
public class GrabberTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        MintGrabber grabber = new MintGrabber(hardwareMap, gamepad2, telemetry);
        MintWrist wrist = new MintWrist(hardwareMap, gamepad2, telemetry, grabber);

        waitForStart();

        telemetry.addData(">", "Starting Program");
        grabber.printPosition();
        telemetry.update();

        while (opModeIsActive()) {

            wrist.run();

            grabber.run();
            grabber.printPosition();

            telemetry.update();

        }
    }
}
