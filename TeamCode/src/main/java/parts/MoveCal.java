package parts;

public class MoveCal {
    public final double dpi, accel, deccel, maxSpeed;

    // don't need accel, right?
    public MoveCal(double dpi, double accel, double deccel, double maxSpeed) {
        this.dpi = dpi;
        this.accel = accel;
        this.deccel = deccel;
        this.maxSpeed = maxSpeed;
    }
}
