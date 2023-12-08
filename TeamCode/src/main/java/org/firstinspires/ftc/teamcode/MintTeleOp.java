package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import parts.MintArm;
import parts.MintLauncher;
import parts.MintWheels;
import parts.MintGrabber;
import parts.MintWrist;

@TeleOp(name = "Mint TeleOp", group = "A")
public class MintTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Gamepad 1
        MintWheels wheels = new MintWheels(hardwareMap, gamepad1, telemetry);
        MintLauncher launcher = new MintLauncher(hardwareMap, gamepad1, telemetry);

        //Gamepad 2
        MintArm arm = new MintArm(hardwareMap, gamepad2, telemetry);
        MintGrabber grabber = new MintGrabber(hardwareMap, gamepad2, telemetry);
        MintWrist wrist = new MintWrist(hardwareMap, gamepad2, telemetry, grabber);

        //Start OpMode
        waitForStart();

        telemetry.addData(">", "Starting Program");
        grabber.printPosition();
        telemetry.update();

        //Run stuff
        while (opModeIsActive()) {
            wheels.run();

            arm.run();

            wrist.run();

            launcher.run();

            grabber.run();
            grabber.printPosition();

            telemetry.update();

        }
    }
}



