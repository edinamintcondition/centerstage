package parts;

public class MotorConfig {
    public final double topSpeed; //degs per sec
    public final double nominalVolt;

    public final double degreeMult;

    public MotorConfig(double topSpeed, double degreeMult, double nominalVolt) {
        this.topSpeed = topSpeed;
        this.degreeMult = degreeMult;
        this.nominalVolt = nominalVolt;
    }

    //    public static final MotorConfig driveMotor = new MotorConfig(890, 280.0 * 4.0 / 360.0, 12);   // 20:1
    public static final MotorConfig driveMotor = new MotorConfig(890, 280.0 * 4.0 / 360.0, 12); // 40:1
    public static final MotorConfig armMotor = new MotorConfig(0, 0, 12);

    public double toDeg(double pos) {
        return pos / degreeMult;
    }

    public int toPos(double deg) {
        return (int) (deg * degreeMult);
    }
}
