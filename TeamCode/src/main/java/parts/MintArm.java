package parts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintArm {

    // Constants
    String motorName = "arm_motor";
    //Sets power to 50%
    double powerLimit = 0.5;

    // Variables
    Gamepad gamepad;
    Telemetry telemetry;
    DcMotor armMotor;

    // Constructor
    public MintArm(HardwareMap hardwareMap, Gamepad gamepadToUse, Telemetry aTelemetry) {
        gamepad = gamepadToUse;

        armMotor = hardwareMap.get(DcMotor.class, motorName);
        armMotor.setDirection(FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = aTelemetry;

        if (gamepad.left_trigger > 0.8) {
            powerLimit = 1.0;
            telemetry.addData(">", "STRONGARM HAHAHA");
        } else {
            powerLimit = 0.5;
            telemetry.addData(">", "normal arm power");
        }
    }

    //Methods
    public void run() {
        // move arm with left joystick on gamepad 2
        double tgtPower = -gamepad.left_stick_y * powerLimit;
        armMotor.setPower(tgtPower);

        telemetry.addData(armMotor.getDeviceName(), tgtPower);
        /* This code *literally* breaks the robot but would work with a stronger wrist servo
        if (isStrongArm || gamepad.right_trigger > 0.8) {
            armMotor.setPower(1.00);
            isStrongArm = true;
            telemetry.addData(">", "STRONGARM HAHAHA");

            if (gamepad.left_trigger > 0.8) {
                armMotor.setPower(0.00);
                isStrongArm = false;
                telemetry.addData(">", "Back to normal!");
            }
        } else {
         */
    }

}
