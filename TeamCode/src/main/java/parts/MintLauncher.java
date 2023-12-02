package parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MintLauncher {
    public final String servoName = "launch_servo";
    public final Servo servo;
    public static final double CLOSED_POSITION = 0.96;
    public static final double LAUNCH_POSITION = 0.5;

    public MintLauncher(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, servoName);
    }


}
