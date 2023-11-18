package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintGrabber {

    //Constants
    String servoName = "grab_servo";
    final double FINAL_POSITION = 0.95;
    final double START_POSITION = 0.4;

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
        myServo.setPosition(FINAL_POSITION); // rotate clockwise
        telemetry.addData(">", "GRAB CLOSED GRRRR");
    }

    public void openGrab() {
        myServo.setPosition(START_POSITION); // rotate counterclockwise
        telemetry.addData(">", "GRAB OPEN COACH");
    }

    public void printPosition() {
        telemetry.addData("Grabber " + myServo.getDeviceName(), myServo.getPosition());
    }

    public void stop() {
        myServo.setPosition(START_POSITION); // stops rotation
    }

    public void start() {

    }

}


