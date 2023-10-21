package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class VisionTest extends LinearOpMode {
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

        WebcamName camera1 = hardwareMap.get(WebcamName.class,"Webcam 1");
        VisionPortal myVisionPort = VisionPortal.easyCreateWithDefaults(camera1,myAprilTagProc);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}