package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm_motor");


        waitForStart();

        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.6);

        while (opModeIsActive()) {
            double max;

            double angle = gamepad1.right_stick_y * 20;
            telemetry.addData("angle", angle);
            telemetry.update();


            armMotor.setTargetPosition((int) angle);


        }
    }
}
