package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import parts.MintArm;
import parts.MintCamera;
import parts.MintWheels;
import parts.MintGrabber;
import parts.MintWrist;

@TeleOp
public class RobotOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Gamepad 1
        MintWheels wheels = new MintWheels(hardwareMap, gamepad1, telemetry);

        //Gamepad 2
        MintArm arm = new MintArm(hardwareMap, gamepad2, telemetry);
        MintWrist wrist = new MintWrist(hardwareMap, gamepad2, telemetry);
        MintGrabber grabber = new MintGrabber(hardwareMap, gamepad2, telemetry);

        //Camera
        MintCamera camera  = new MintCamera(hardwareMap,  telemetry);

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

            grabber.run();
            grabber.printPosition();

            camera.run();

            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad2.dpad_down) {
                camera.stopStreaming();
                telemetry.addData(">", "camara not streaming :(");
            } else if (gamepad2.dpad_up) {
                camera.resumeStreaming();
                telemetry.addData(">", "camara streaming :D");
            }

            telemetry.update();

            sleep(20);
        }
        camera.stopStreaming();
    }
}



