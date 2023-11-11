package parts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintGrabber {

    //Constants
    String servoName = "grab_servo";
    final double FINAL_POSITION = 0.5;
    final double START_POSITION = 0;

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
            pressedButton = "left";
            rotateClock(); // rotate clockwise;
        } else if (gamepad.right_bumper) {
            pressedButton = "right";
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


    public void printPosition() {
        telemetry.addData("Servo Position", "%5.2f", myServo.getPosition());
    }

    public void stop() {
        myServo.setPosition(START_POSITION); // stops rotation
    }

    public void start() {

    }

}


