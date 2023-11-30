package org.firstinspires.ftc.teamcode;

public class MotorConfig {
    public final double topSpeed;

    public final double degreeMult;

    public MotorConfig(double topSpeed, double degreeMult) {
        this.topSpeed = topSpeed;
        this.degreeMult = degreeMult;
    }

    public static final MotorConfig driveMotor = new MotorConfig(0, 280.0 * 4.0 / 360.0);
    public static final MotorConfig armMotor = new MotorConfig(0, 0);

    public double toDeg(double pos) {
        return pos / degreeMult;
    }

    public int toPos(double deg) {
        return (int)(deg * degreeMult);
    }
}
