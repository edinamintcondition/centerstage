package parts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintWrist {

    // Constants
    String servoName = "wrist_servo";
    final double FINAL_POSITION = 1;
    final double START_POSITION = 0;
    final double FLAT = 0.8  ;

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
        String pressedButton = "nothing";
        // move servo
        if (gamepad.a) {
            pressedButton = "'a'";
            positionOne(); // fully extend
        } else if (gamepad.b) {
            pressedButton = "'b'";
            positionZero(); // retract
        } else if (gamepad.x) {
            pressedButton = "'x'";
            positionTwo(); // extend until flat with the ground
        } else if (gamepad.y)
            pressedButton = "'y'";

        telemetry.addData(">", pressedButton + " is pressed :D");
    }


    public void positionZero() {
        myServo.setPosition(START_POSITION); // retracts
        telemetry.addData(">","wrist in..");
    }

    public void positionOne() {
        myServo.setPosition(FINAL_POSITION); // extends
        telemetry.addData(">","wrist out!");
    }

    public void positionTwo() {
        myServo.setPosition(FLAT); // Is parallel to the floor when are is fully down
        telemetry.addData(">","wrist flat");
    }

    public void positionTea() {}

}
