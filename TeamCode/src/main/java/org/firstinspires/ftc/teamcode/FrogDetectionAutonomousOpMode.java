package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import parts.MintWheels;

@Disabled
@TeleOp
public class FrogDetectionAutonomousOpMode extends LinearOpMode {
    OpenCvWebcam webcam;
    FrogDetectionPipeline pipeline;
    FrogDetectionPipeline.FrogPosition snapshotAnalysis = FrogDetectionPipeline.FrogPosition.LEFT; // default

    @Override
    public void runOpMode() {
        MintWheels wheels = new MintWheels(hardwareMap, gamepad1, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new FrogDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();

        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis) {
            case LEFT: {
                /* Your autonomous code */
//                wheels.turnLeft(opModeIsActive());
                break;
            }

            case RIGHT: {
                /* Your autonomous code */
//                wheels.turnRight(opModeIsActive());
                break;
            }

            case CENTER: {
                /* Your autonomous code*/
//                wheels.moveForward(opModeIsActive());
                break;
            }
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

}