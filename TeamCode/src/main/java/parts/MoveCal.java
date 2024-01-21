package parts;

public class MoveCal {
    public final double dpi, accel, maxSpeed;
    public double deccel;

    public MoveCal(double dpi, double accel, double deccel, double maxSpeed) {
        this.dpi = dpi;
        this.accel = accel;
        this.deccel = deccel;
        this.maxSpeed = maxSpeed;
    }
}
