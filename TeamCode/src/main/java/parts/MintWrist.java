package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintWrist {

    // Constants
    String servoName = "wrist_servo";
    final double FINAL_POSITION = 0.5;
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
        if (gamepad.dpad_up) {
            pressedButton = "dpad up";
            rotateClock(); // rotate clockwise;
        } else if (gamepad.dpad_down) {
            pressedButton = "dpad down";
            rotateCounterClock(); // rotate counterclockwise
//        } else {
            //          stop(); // stops rotation
            //        pressedButton = "nothing";
        }

        telemetry.addData(">", pressedButton + " is pressed :D");
    }

    public void rotateClock() {
        myServo.setPosition(FINAL_POSITION); // rotate clockwise
    }

    public void rotateCounterClock() {
        myServo.setPosition(START_POSITION); // rotate counterclockwise
    }


}
