package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import parts.MintGrabber;
import parts.MintLauncher;

//@Disabled

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String button = "nothing";
        MintLauncher launcher = new MintLauncher(hardwareMap, gamepad1, telemetry);

        //Start OpMode
        waitForStart();


        //Run stuff
        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                button = "100";
                launcher.servo.setPosition(MintLauncher.CLOSED_POSITION);
            } else if (gamepad2.right_bumper) {
                button = "0";
                launcher.servo.setPosition(MintLauncher.LAUNCH_POSITION);
            }

            telemetry.addData(">>", button);
            telemetry.update();

        }

    }

}
