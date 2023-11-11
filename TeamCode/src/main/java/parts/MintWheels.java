package parts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.util.Arrays.asList;
import static java.util.Collections.max;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MintWheels {

    // Constants
    //Sets power to 60%
    double powerLimit = 0.60;

    // Variables
    Gamepad gamepad;
    Telemetry telemetry;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    // Constructor
    public MintWheels(HardwareMap hardwareMap, Gamepad gamepadToUse, Telemetry aTelemetry) {
        gamepad = gamepadToUse;

        leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        leftFront.setDirection(FORWARD);

        rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        rightFront.setDirection(FORWARD);

        leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        leftBack.setDirection(REVERSE);

        rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");
        rightBack.setDirection(REVERSE);

        telemetry = aTelemetry;
    }

    //Methods
    public void run() {
        //These turn a joystick input into a number
        double axial = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;
        double yaw = gamepad.right_stick_x;

        //Calculates the wheel direction
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = max(asList(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower));

        if (gamepad.right_bumper) {
            powerLimit = 1.0;
        }

        controlWheel(leftFront, leftFrontPower, max);
        controlWheel(rightFront, rightFrontPower, max);
        controlWheel(leftBack, leftBackPower, max);
        controlWheel(rightBack, rightBackPower, max);
    }

    private void controlWheel(DcMotor motor, double tgtPower, double maxPower) {
        if (maxPower > powerLimit) {
            tgtPower *= powerLimit / maxPower;
        }

        // move motor
        motor.setPower(tgtPower);


        telemetry.addData(motor.getDeviceName(), tgtPower);
    }

    public void oneTileForward () {

    }

}
