package parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MintMotor implements Movement{

    DcMotor myMotor;
    public MintMotor(HardwareMap hardwareMap){
        myMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
    }

    @Override
    public String control(Gamepad gamepad) {

        // move motor
        double tgtPower = -gamepad.left_stick_y;
        myMotor.setPower(tgtPower);

        return "" + tgtPower;
    }
}
