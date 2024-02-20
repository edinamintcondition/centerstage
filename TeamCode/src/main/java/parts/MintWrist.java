package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintWrist {

    // Constants
    String servoName = "wrist_servo";
    public static final double DROP_POSITION = 0.58;
    public static final double RETRACTED_POS = 0.3;
    public static final double FLAT_POSITION = 0.45;
    public static final double EXTENDED_POS = 0.695;
    double wristPosition = 0.0;

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
            positionTwo(); // pixel drop pos
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
            positionThree(); // fully extends
        }

        wristPosition = wristPosition + (-gamepad.right_stick_y / 10);

        myServo.setPosition(wristPosition);

        if (gamepad.right_stick_y > 0 || gamepad.right_stick_y < 0) {
            telemetry.addData(">", "wrist fine tune being used");
        }

        telemetry.addData(">", pressedButton + " is pressed :D");
        telemetry.addData("The Wrist Servo position:", myServo.getPosition());
    }


    public void positionZero() {
        wristPosition = RETRACTED_POS; // retracts
        telemetry.addData(">","retracted");
    }

    public void positionOne() {
        wristPosition = FLAT_POSITION; // Is parallel to the floor when are is fully down
        telemetry.addData(">","wrist flat!");
    }

    public void positionTwo() {
        wristPosition = DROP_POSITION; // gets wrist at 30 degrees
        telemetry.addData(">","pixel drop pos!");
    }

    public void positionThree() {
        wristPosition = EXTENDED_POS; // fully extends
        telemetry.addData(">","i'm all out");
    }

    public void positionTea() {}

}
