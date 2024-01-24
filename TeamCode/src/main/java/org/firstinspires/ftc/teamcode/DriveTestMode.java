package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import parts.MintDrive;

@Autonomous
@Disabled
public class DriveTestMode extends LinearOpMode {
    /*Set ERR_MULT and DRAG_MULT = 0, and calibrate.
    Grab pos at end of acceleration, and compare to degreeStopAccel
    ratio used to change ACCEL_MULT
    once start cruising, start accelerometer (if you hit 0 then stop)
    also get stats on torquefrac used in cruise mode
    Use LinearFuncFitter to find DRAG_MULT for cruise
    do something similar for deceleration*/
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
