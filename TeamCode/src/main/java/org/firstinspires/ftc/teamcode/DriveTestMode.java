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

    private void test(double targetPos, boolean strafe) {
        md.setDriveDist(targetPos, strafe);

        DynamicParams adp = md.getActiveDynamicParams();
        adp.startCalibration();

        telemetry.addData("DP", adp);
        telemetry.addData("pos", "target=%.1f", targetPos);
        telemetry.update();

        while (opModeIsActive()) {
            boolean done = md.runDrive(false);
            if (done) break;
        }

        double actualPos = md.getDeg(strafe);

        adp.finishCalibration();

        telemetry.addData("DP", adp);
        telemetry.addData("pos", "target=%.1f, actual=%.1f", targetPos, actualPos);
        telemetry.update();
        sleep(5000);
    }
}
