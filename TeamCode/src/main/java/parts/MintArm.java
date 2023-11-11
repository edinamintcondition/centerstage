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
    }

    //Methods
    public void run() {
        // move arm with left joystick on gamepad 2
        double tgtPower = -gamepad.left_stick_y * powerLimit;
        armMotor.setPower(tgtPower);

        telemetry.addData(armMotor.getDeviceName(), tgtPower);
    }

}
