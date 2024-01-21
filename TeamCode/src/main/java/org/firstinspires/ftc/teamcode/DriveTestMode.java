package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import parts.Calibrator;
import parts.MintDrive;
import parts.MoveCal;

@Autonomous
public class DriveTestMode extends LinearOpMode {
    private MintDrive md;

    protected boolean strafe;


    // how to do dpi calibration: ...
    @Override
    public void runOpMode() {
        md = new MintDrive(hardwareMap);
        waitForStart();

        test(40);
        sleep(1000);
        test(-40);

        Calibrator cal = new Calibrator(20.5, md.getActiveMoveCal().deccel);

        for (int trial = 0; trial < 10; trial++) {
            md.getActiveMoveCal().deccel = cal.getGuess();
            if (md.getActiveMoveCal().deccel >= 0)
                break;

            test(20);
            cal.updateGuess(md.getPos());
            sleep(2000);

            test(-20);
        }
    }

    private void test(double targetDist) {
        md.resetPos();
        md.preRun(targetDist, strafe);

        telemetry.addData("deccel", "deccel=%.1f", md.getActiveMoveCal().deccel);
        telemetry.addData("pos", "target=%.1f", targetDist);
        telemetry.update();
        sleep(500);

        while (opModeIsActive()) {
            boolean done = md.run();
            if (done) break;
        }

        double actualPos = md.getPos();
        double actualDeg = md.getDeg();

        telemetry.addData("deccel", "deccel=%.1f", md.getActiveMoveCal().deccel);
        telemetry.addData("pos", "target=%.1f, actual=%.1f, actualDeg=%.1f", targetDist, actualPos, actualDeg);
        telemetry.update();
        sleep(500);
    }
}
