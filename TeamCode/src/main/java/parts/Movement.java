package parts;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface Movement {
    String control(Gamepad gamepad);
}
