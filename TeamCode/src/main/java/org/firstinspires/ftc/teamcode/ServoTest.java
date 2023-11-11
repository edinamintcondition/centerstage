package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import parts.MintGrabber;

@TeleOp
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MintGrabber grabber = new MintGrabber(hardwareMap, gamepad2, telemetry);
        String button = "nothing";

        //Start OpMode
        waitForStart();

        telemetry.addData(">", "Starting Program");
        grabber.printPosition();
        telemetry.update();

        //Run stuff
        while (opModeIsActive()) {

            grabber.run();
            grabber.printPosition();

            if (gamepad2.left_bumper) {
                button = "closing";
                wait((long)0.5);
            } else if (gamepad2.right_bumper) {
                button = "opening";
                wait((long) 0.5);
            } else {
                button = "being silly";
                wait((long)0.5);
            }

            telemetry.addData("<", "the claw is " + button + " >:3");
            telemetry.update();

        }

    }

}
