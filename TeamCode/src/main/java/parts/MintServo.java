package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintServo implements Rotator, Movement {
    Servo myServo;
    public MintServo(HardwareMap hardwareMap){
        myServo = hardwareMap.get(Servo.class, "test_servo");
    }
    @Override
    public void rotateClock() {
        myServo.setPosition(0); // rotate clockwise
    }

    @Override
    public void rotateCounterClock() {
        myServo.setPosition(1); // rotate counterclockwise
    }

    @Override
    public void stop() {
        myServo.setPosition(0.5); // stops rotation
    }

    @Override
    public void start() {

    }

    @Override
    public void printPosition(Telemetry telemetry) {
        telemetry.addData("Servo Position", "%5.2f", myServo.getPosition());
    }

    @Override
    public String control(Gamepad gamepad) {
        String pressedButton;
        // move servo
        if (gamepad.dpad_left) {
            pressedButton = "left";
            rotateClock(); // rotate clockwise
        } else if (gamepad.dpad_right) {
            pressedButton = "right";
            rotateCounterClock(); // rotate counterclockwise
        } else {
            stop(); // stops rotation
            pressedButton = "nothing";
        }
        return pressedButton;
    }
}


