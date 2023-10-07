package parts;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Rotator {
    void rotateClock();
    void rotateCounterClock();
    void stop();
    void start();
    void printPosition(Telemetry telemetry);
}
