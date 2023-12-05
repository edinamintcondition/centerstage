package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintGrabber {

    //Constants
    String servoNameL = "grab_servo_L";
    String servoNameR = "grab_servo_R";
    public static final double CLOSED_POSITION = 0.96;
    public static final double OPEN_POSITION = 0.5;

    public static void init(Servo s) {
    }

    //Variables
    Servo myServoL;
    Servo myServoR;
    Gamepad gamepad;
    Telemetry telemetry;

    //Constructor
    public MintGrabber(HardwareMap hardwareMap, Gamepad gamepadToUse, Telemetry aTelemetry) {
        gamepad = gamepadToUse;

        myServoL = hardwareMap.get(Servo.class, servoNameL);
        myServoR = hardwareMap.get(Servo.class, servoNameR);

        telemetry = aTelemetry;
    }

    //Methods
    public void run() {
        init(myServoL);
        init(myServoR);

        String pressedButton = null;

        // move servo

        // both grabbers
        if (gamepad.left_bumper && !gamepad.dpad_right && !gamepad.dpad_left) {
            pressedButton = "left bumper";
            closeGrabLR(); // rotate clockwise;
        } else if (gamepad.right_bumper && !gamepad.dpad_right && !gamepad.dpad_left) {
            pressedButton = "right bumper";
            openGrabLR(); // rotate counterclockwise
        }

        //left grabber
        if (gamepad.left_bumper && !gamepad.dpad_right && gamepad.dpad_left) {
            pressedButton = "left bumper + left dpad";
            closeGrabL();
        } else if (gamepad.right_bumper && !gamepad.dpad_right && gamepad.dpad_left) {
            pressedButton = "right bumper + left dpad";
            openGrabL();
        }

        //right grabber
        if (gamepad.right_bumper && gamepad.dpad_right && !gamepad.dpad_left) {
            pressedButton = "left bumper + right dpad";
            closeGrabR();
        } else if (gamepad.right_bumper && gamepad.dpad_right && !gamepad.dpad_left) {
            pressedButton = "right bumper + left dpad";
            openGrabR();
        }

        telemetry.addData(">", pressedButton + " is pressed :D");
    }

    //both grabbers
    public void openGrabLR() {
        myServoL.setPosition(CLOSED_POSITION); // close grabbers
        myServoR.setPosition(OPEN_POSITION);
        telemetry.addData(">", "Both grabs opening :O");
    }
    public void closeGrabLR() {
        myServoL.setPosition(OPEN_POSITION); // open grabbers
        myServoR.setPosition(CLOSED_POSITION);
        telemetry.addData(">", "Both grabs closing :D");
    }

    //left grabber
    public void closeGrabL() {
        myServoL.setPosition(CLOSED_POSITION); //Closes left grabber
        telemetry.addData(">", "Left grabb closing");
    }
    public void openGrabL() {
        myServoL.setPosition(OPEN_POSITION); //Opens left grabber
        telemetry.addData(">", "Left grabb opening");
    }

    //right grabber
    public void closeGrabR() {
        myServoR.setPosition(OPEN_POSITION); //Closes right grabber
        telemetry.addData(">", "right grab closing");
    }
    public void openGrabR() {
        myServoR.setPosition(CLOSED_POSITION); //Opens right grabber
        telemetry.addData(">", "right grab opening");
    }

    public void printPosition() {
        telemetry.addData("Grabber " + myServoL.getDeviceName(), myServoL.getPosition());
    }

    public void stop() {
        myServoL.setPosition(OPEN_POSITION); // stops rotation
    }

    public void start() {

    }

}


