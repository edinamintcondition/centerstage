package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import parts.MintDrive;

@Autonomous
public class RevTestMode extends LinearOpMode {
    // how to do dpi calibration: ...
    @Override
    public void runOpMode() {
        MintDrive md = new MintDrive(hardwareMap);
        waitForStart();
        md.resetPos();
        md.preRun(-20, false);
        while (opModeIsActive()) {
            boolean done = md.run();
            telemetry.addData("drive", md.moveString());
            telemetry.addData("curr", "speed %.1f, deg=%.1f", md.getSpeed(), md.getDeg());
            telemetry.update();
            if (done) break;
        }
    }
}
