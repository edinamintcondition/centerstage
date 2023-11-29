package org.firstinspires.ftc.teamcode;

public class MotorConfig {
    public final double topSpeed;

    public final double degreeMult;

    public MotorConfig(double topSpeed, double degreeMult) {
        this.topSpeed = topSpeed;
        this.degreeMult = degreeMult;
    }

    public static final MotorConfig driveMotor = new MotorConfig(0, 0);
    public static final MotorConfig armMotor = new MotorConfig(0, 0);
}
