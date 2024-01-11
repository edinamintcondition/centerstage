package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import parts.MintDrive;

@Autonomous
public class DriveTestMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        MintDrive md = new MintDrive(hardwareMap, telemetry);
        waitForStart();

        md.setDriveDist(60, false);

        while (opModeIsActive()) {
            boolean done = md.runDrive(false);
            if (done) break;
        }
    }
}
