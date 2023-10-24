package parts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintWrist {

    // Constants
    String servoName = "test_servo";

    // Variables
    Gamepad gamepad;
    Telemetry telemetry;
    CRServo myServo;

    // Constructor
    public MintWrist(HardwareMap hardwareMap, Gamepad gamepadToUse, Telemetry aTelemetry) {
        gamepad = gamepadToUse;

        myServo = hardwareMap.get(CRServo.class, servoName);

        telemetry = aTelemetry;
    }

    //Methods
    public void run() {
        // move servo
        double tgtPower = -gamepad.left_stick_y;
        myServo.setPower(tgtPower);
    }
}
