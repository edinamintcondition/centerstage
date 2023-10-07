import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp
public class RobynTest extends LinearOpMode {
    @Override
    public void runOpMode() {


        waitForStart();
        Servo leftFrontDrive = hardwareMap.get(Servo.class, "test_servo");

                leftFrontDrive.setDirection(Servo.Direction.FORWARD);
                RobotLog.d("User runOpModeMethod exited");
            if (gamepad2.a) {
                RobotLog.d("User runOpModeMethod exited");

                while (opModeIsActive()) {
                    double leftFrontPower = 0.5;

                    leftFrontDrive.setPosition(leftFrontPower);
                }
            }
    }
}
