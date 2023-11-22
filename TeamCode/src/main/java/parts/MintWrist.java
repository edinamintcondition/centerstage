package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintWrist {

    // Constants
    String servoName = "wrist_servo";
    final double FINAL_POSITION = 0.7;
    final double ZERO_POSITION = 0;
    final double FLAT_POSITION = 0.6;

    // Variables
    Gamepad gamepad;
    Telemetry telemetry;
    MintGrabber grabber;
    Servo myServo;

    // Constructor
    public MintWrist(HardwareMap hardwareMap, Gamepad gamepadToUse, Telemetry aTelemetry, MintGrabber aGrabber) {
        gamepad = gamepadToUse;

        myServo = hardwareMap.get(Servo.class, servoName);

        telemetry = aTelemetry;

        grabber = aGrabber;
    }

    //Methods
    public void run() {
        String pressedButton = "nothing";
        // move servo
        if (gamepad.a) {
            grabber.closeGrab();
            pressedButton = "'a'";
            positionTwo(); // fully extend
        } else if (gamepad.b) {
            grabber.closeGrab();
            pressedButton = "'b'";
            positionZero(); // retract
        } else if (gamepad.x) {
            grabber.closeGrab();
            pressedButton = "'x'";
            positionOne(); // extend until flat with the ground
        } else if (gamepad.y) {
            pressedButton = "'y'";
        }

        telemetry.addData(">", pressedButton + " is pressed :D");
        telemetry.addData("The Wrist Servo position:", myServo.getPosition());
    }


    public void positionZero() {
        myServo.setPosition(ZERO_POSITION); // retracts
        telemetry.addData(">","wrist in..");
    }

    public void positionOne() {
        myServo.setPosition(FINAL_POSITION); // extends
        telemetry.addData(">","wrist out!");
    }

    public void positionTwo() {
        myServo.setPosition(FLAT_POSITION); // Is parallel to the floor when are is fully down
        telemetry.addData(">","wrist flat");
    }

    public void positionTea() {}

}
