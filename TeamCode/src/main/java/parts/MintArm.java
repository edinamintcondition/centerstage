package parts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintArm {

    // Constants
    String motorName = "arm_motor";
    //Sets power to 50%
    double normPower = 0.5;

    // Variables
    Gamepad gamepad;
    Telemetry telemetry;
    DcMotor armMotor;
    boolean isStrongArm;

    // Constructor
    public MintArm(HardwareMap hardwareMap, Gamepad gamepadToUse, Telemetry aTelemetry) {
        gamepad = gamepadToUse;

        armMotor = hardwareMap.get(DcMotor.class, motorName);
        armMotor.setDirection(FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = aTelemetry;
    }

    //Methods
    public void run() {

        if (isStrongArm || gamepad.right_trigger > 0.8) {
            armMotor.setPower(1.00);
            isStrongArm = true;
            telemetry.addData(">", "STRONGARM HAHAHAHA");

            if (gamepad.left_trigger > 0.8) {
                armMotor.setPower(0.00);
                isStrongArm = false;
                telemetry.addData(">", "Back to normal!!");
            }

        } else {

            double armPower = 0.0;

            if (gamepad.left_stick_y >= 0) {
                armPower = gamepad.left_stick_y * normPower;
                armMotor.setPower(armPower);
            } else if (gamepad.left_stick_y < 0) {
                armPower = (gamepad.left_stick_y - 0.5) * normPower;
                armMotor.setPower(armPower);
            }

            telemetry.addData(">", "normal arm power D':");

            telemetry.addData("Arm " + armMotor.getDeviceName(), "%f %d", armPower, armMotor.getCurrentPosition());
        }
    }
}

