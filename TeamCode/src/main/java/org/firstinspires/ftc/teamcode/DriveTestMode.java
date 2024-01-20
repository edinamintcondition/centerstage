package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import parts.DynamicParams;
import parts.MintDrive;

@Autonomous
public class DriveTestMode extends LinearOpMode {
    private MintDrive md;

    @Override
    public void runOpMode() {
        md = new MintDrive(hardwareMap, telemetry);
        waitForStart();

        int dir = 1;
        for (int trial = 0; trial < 6; trial++) {
            test(60 * dir, false);
            dir = -dir;
        }
    }

    private void test(double targetDist, boolean strafe) {
        md.preRun(targetDist, strafe);

        md.addTelemetry();
        telemetry.addData("pos", "target=%.1f", targetDist);
        telemetry.update();
        sleep(5000);

        while (opModeIsActive()) {
            boolean done = md.run();
            if (done) break;
        }

        double actualPos = md.getPos();

        md.addTelemetry();
        telemetry.addData("pos", "target=%.1f, actual=%.1f", targetDist, actualPos);
        telemetry.update();
        sleep(5000);
    }
}
