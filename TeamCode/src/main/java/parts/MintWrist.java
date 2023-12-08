package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintWrist {

    // Constants
    String servoName = "wrist_servo";
    public static final double DROP_POSITION = 0.7;
    public static final double RETRACTED_POS = 0.425;
    public static final double FLAT_POSITION = 0.58;
    public static final double HANG_POSITION = 0.8;

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
            grabber.closeGrabLR();
            pressedButton = "'a'";
            positionTwo(); // fully extend
        } else if (gamepad.b) {
            grabber.closeGrabLR();
            pressedButton = "'b'";
            positionZero(); // retract
        } else if (gamepad.x) {
            grabber.closeGrabLR();
            pressedButton = "'x'";
            positionOne(); // extend until flat with the ground
        } else if (gamepad.y) {
            pressedButton = "'y'";
            positionThree();
        }

        telemetry.addData(">", pressedButton + " is pressed :D");
        telemetry.addData("The Wrist Servo position:", myServo.getPosition());
    }


    public void positionZero() {
        myServo.setPosition(RETRACTED_POS); // retracts
        telemetry.addData(">","retracted");
    }

    public void positionOne() {
        myServo.setPosition(DROP_POSITION); // extends
        telemetry.addData(">","wrist flat!");
    }

    public void positionTwo() {
        myServo.setPosition(FLAT_POSITION); // Is parallel to the floor when are is fully down
        telemetry.addData(">","pixel drop pos!");
    }

    public void positionThree() {
        myServo.setPosition(HANG_POSITION); // for hanging
        telemetry.addData(">","hanging ready");
    }

    public void positionTea() {}

}
