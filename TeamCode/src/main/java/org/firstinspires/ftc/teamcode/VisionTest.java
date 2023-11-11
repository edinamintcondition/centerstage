package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
@Disabled
public class VisionTest extends LinearOpMode {
    private final double[] allmx = {29.381, 35.381, 41.381, 100.0435, 106.0435, 112.0435};
    private final double[] allmy = {132.492908, 132.492908, 132.492908, 132.492908, 132.492908, 132.492908, 0, 0, 0, 0};

    @Override
    public void runOpMode() {
        AprilTagProcessor.Builder myAprilTagProcBuilder;
        AprilTagProcessor myAprilTagProc;

        myAprilTagProcBuilder = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true);

        myAprilTagProc = myAprilTagProcBuilder.build();

        WebcamName camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal myVisionPort = VisionPortal.easyCreateWithDefaults(camera1, myAprilTagProc);

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> currentDetections = myAprilTagProc.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id <= 6) {
                    int i = detection.id - 1;

                    double mx = allmx[i];
                    double my = allmy[i];

                    double mc = Math.sqrt((detection.ftcPose.x * detection.ftcPose.x) + (detection.ftcPose.y * detection.ftcPose.y));

                    double dx = Math.sin(Math.toRadians(detection.ftcPose.yaw));
                    double dy = Math.cos(Math.toRadians(detection.ftcPose.yaw));

                    double qc = dx * mc;
                    double rx = qc + mx;

                    double mq = dy * mc;
                    double ry = my - mq;

                    telemetry.addData("ID", detection.id);
                    telemetry.addData("cx", detection.ftcPose.x);
                    telemetry.addData("cy", detection.ftcPose.y);
                    telemetry.addData("yaw", detection.ftcPose.yaw);
                    telemetry.addData("rx", rx);
                    telemetry.addData("ry", ry);
                    telemetry.addData("dx", dx);
                    telemetry.addData("dy", dy);
                    break;
                }
            }
            telemetry.update();
        }
    }
}