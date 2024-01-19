package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    private void test(double targetPos, boolean strafe) {
        md.setDriveDist(targetPos, strafe);

        while (opModeIsActive()) {
            boolean done = md.runDrive(false);
            if (done) break;
        }

        telemetry.addData("DP", md.getActiveDynamicParams());
        telemetry.update();
        sleep(5000);
    }
}
