package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintWrist {

    // Constants
    String servoName = "wrist_servo";
    final double FINAL_POSITION = 1;
    final double START_POSITION = 0;

    // Variables
    Gamepad gamepad;
    Telemetry telemetry;
    Servo myServo;

    // Constructor
    public MintWrist(HardwareMap hardwareMap, Gamepad gamepadToUse, Telemetry aTelemetry) {
        gamepad = gamepadToUse;

        myServo = hardwareMap.get(Servo.class, servoName);

        telemetry = aTelemetry;
    }

    //Methods
    public void run() {
        String pressedButton = null;
        // move servo
        if (gamepad.a) {
            pressedButton = "'a'";
            positionOne(); // extend;
        } else if (gamepad.b) {
            pressedButton = "'b'";
            positionZero(); // retract
        } else if (gamepad.x) {
            pressedButton = "'x'";
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

    public void positionTwo() {}

    public void positionTea() {}

}
