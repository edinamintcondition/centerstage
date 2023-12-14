package parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintLauncher {
    public final String servoName = "launch_servo";
    public final Servo servo;
    public static final double CLOSED_POSITION = 0.96;
    public static final double LAUNCH_POSITION = 0.5;
    Gamepad gamepad;
    Telemetry telemetry;
    ElapsedTime t;

    public MintLauncher(HardwareMap hardwareMap, Gamepad aGamepad, Telemetry aTelemetry) {
        gamepad = aGamepad;

        servo = hardwareMap.get(Servo.class, servoName);

        telemetry = aTelemetry;
    }

    public void run(){
        if (t == null) {
            t = new ElapsedTime();
        }

        if (t.seconds() > 90) {
            if (gamepad.dpad_up) {
                launch();
                telemetry.addData(">", "dpad up");
            }
            if (gamepad.dpad_down) {
                reset();
                telemetry.addData(">", "dpad down");
            }
        }
    }

    private void  launch(){
        servo.setPosition(LAUNCH_POSITION);
        telemetry.addData(">", "WATCH OUT I'M LAUNCHING");
    }

    private void reset(){
        servo.setPosition(CLOSED_POSITION);
        telemetry.addData(">", "Launcher is resetting :3");
    }


}
