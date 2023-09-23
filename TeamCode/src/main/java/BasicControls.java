
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class BasicControls extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            double max;

            double axial    = -gamepad1.left_stick_y;
            double lateral  = gamepad1.left_stick_x;
            double yaw      = gamepad1.right_stick_x;

            double leftFrontPower   = axial + lateral + yaw;
            double rightFrontPower  = axial - lateral - yaw;
            double leftBackPower    = axial - lateral + yaw;
            double rightBackPower   = axial + lateral - yaw;

            // max power of any motor, either direction
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }
}
