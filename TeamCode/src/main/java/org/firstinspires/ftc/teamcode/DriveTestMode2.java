package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import parts.Calibrator;
import parts.DriveDirection;
import parts.MintDrive;
import parts.MoveCal;

@Autonomous
public class DriveTestMode2 extends LinearOpMode {
    // how to do dpi calibration: drive, read actualDeg from Driver Station, measure distance with tape measure,divide actualDeg by measured distance
    @Override
    public void runOpMode() {
        MintDrive md = new MintDrive(hardwareMap);
        waitForStart();
        md.resetPos();
        md.preRun(20, DriveDirection.Diagonal);
        while (opModeIsActive()) {
            telemetry.addData("drive", "speed=%.1f", md.getSpeed());
            for (int i = 0; i < 4; i++)
                telemetry.addData("motor", "%d %s", i, md.get(i));
            telemetry.update();

            boolean done = md.run();
            if (done) break;
        }
    }
}
