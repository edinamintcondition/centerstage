package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import parts.MintGrabber;
import parts.MintWrist;

@TeleOp
//@Disabled
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MintGrabber grabber = new MintGrabber(hardwareMap, gamepad2, telemetry);
        MintWrist wrist = new MintWrist(hardwareMap, gamepad2, telemetry);
        String button = "nothing";

        //Start OpMode
        waitForStart();

        telemetry.addData(">", "Starting Program");
        grabber.printPosition();
        telemetry.update();

        //Run stuff
        while (opModeIsActive()) {

            telemetry.addData("<", "press a for grabber, press b for wrist");
            telemetry.update();

            if (gamepad1.a || gamepad2.a) {

                telemetry.addData("<", "this is the grabber program");
                telemetry.update();

                while (opModeIsActive()) {

                    grabber.run();
                    grabber.printPosition();

                    if (gamepad2.left_bumper) {
                        button = "closing";
                    } else if (gamepad2.right_bumper) {
                        button = "opening";
                    } else {
                        button = "being silly";
                    }

                    telemetry.addData("<", "the claw is " + button + " >:3");
                    telemetry.update();
                }

            } else if (gamepad1.b || gamepad2.b) {

                telemetry.addData("<", "this is the grabber program");
                telemetry.update();

                while (opModeIsActive()) {
                    wrist.run();

                }
            }
        }
    }
}
