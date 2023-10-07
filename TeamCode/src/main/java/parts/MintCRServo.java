package parts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.zip.CRC32;

public class MintCRServo implements Movement, Rotator {
    CRServo myServo;

    public void MintServo(HardwareMap hardwareMap) {
        myServo = hardwareMap.get(CRServo.class, "test_servo");
    }

    @Override
    public void rotateClock() {
        myServo.setPower(1); // rotate clockwise
    }

    @Override
    public void rotateCounterClock() {
//        myServo.setPosition(1); // rotate counterclockwise
    }

    @Override
    public void stop() {
//        myServo.setPosition(0.5); // stops rotation
    }

    @Override
    public void start() {

    }

    @Override
    public void printPosition(Telemetry telemetry) {
//        telemetry.addData("Servo Position", "%5.2f", myServo.getPosition());
    }


    @Override
    public String control(Gamepad gamepad) {
        String pressedButton;
        // move servo
        double tgtPower = -gamepad.left_stick_y;
        myServo.setPower(tgtPower);

        return "" + tgtPower;
    }
}
