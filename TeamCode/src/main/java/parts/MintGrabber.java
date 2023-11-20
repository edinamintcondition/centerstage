package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintGrabber {

    //Constants
    String servoName = "grab_servo";
    public static final double CLOSED_POSITION = 0.96;
    public static final double OPEN_POSITION = 0.5;

    public static void init(Servo s) {
    }

    //Variables
    Servo myServo;
    Gamepad gamepad;
    Telemetry telemetry;

    //Constructor
    public MintGrabber(HardwareMap hardwareMap, Gamepad gamepadToUse, Telemetry aTelemetry) {
        gamepad = gamepadToUse;

        myServo = hardwareMap.get(Servo.class, servoName);

        telemetry = aTelemetry;
    }

    //Methods
    public void run() {
        init(myServo);

        String pressedButton = null;
        // move servo
        if (gamepad.left_bumper) {
            pressedButton = "left bumper";
            closeGrab(); // rotate clockwise;
        } else if (gamepad.right_bumper) {
            pressedButton = "right bumper";
            openGrab(); // rotate counterclockwise
//        } else {
            //          stop(); // stops rotation
            //        pressedButton = "nothing";
        }

        telemetry.addData(">", pressedButton + " is pressed :D");
    }

    public void closeGrab() {
        myServo.setPosition(CLOSED_POSITION); // rotate clockwise
        telemetry.addData(">", "GRAB CLOSED GRRRR");
    }

    public void openGrab() {
        myServo.setPosition(OPEN_POSITION); // rotate counterclockwise
        telemetry.addData(">", "GRAB OPEN COACH");
    }

    public void printPosition() {
        telemetry.addData("Grabber " + myServo.getDeviceName(), myServo.getPosition());
    }

    public void stop() {
        myServo.setPosition(OPEN_POSITION); // stops rotation
    }

    public void start() {

    }

}


