import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DDClawTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        DcMotor motorTest = hardwareMap.get(DcMotor.class, "front_left_motor");
        Servo testServo = hardwareMap.get(Servo.class, "test_servo");

        telemetry.addData(">", "Starting Program");

        double tgtPower;
        while (opModeIsActive()) {
            // move motor
            tgtPower = -gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);

            // move servo
            if (gamepad1.a) {
                telemetry.addData(">", "button 'A' pressed :D");
                testServo.setPosition(0); // 0 degrees
            } else if (gamepad1.b) {
                telemetry.addData(">", "button 'B' pressed :D");
                testServo.setPosition(1); // 180 degrees
            }

            telemetry.update();
        }
    }

}

