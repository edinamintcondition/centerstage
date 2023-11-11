package parts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.util.Arrays.asList;
import static java.util.Collections.max;
import static java.util.Collections.min;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagPositioning;
import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.vision.VisionPortal;

public class MintWheels {

    // Constants
    //Sets power to 60%
    double forwardPowerLimit = 0.60;
    double backwardPowerLimit = -0.60;

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
        double min = min(asList(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower));

        if (gamepad.right_bumper) {
            forwardPowerLimit = 1.0;
            backwardPowerLimit = -1.0;
            telemetry.addData(">", "TURBO LETS GOOOO");
        } else {
            forwardPowerLimit = 0.6;
            backwardPowerLimit = -0.6;
            telemetry.addData( ">", "normal speed :b");
        }

        controlWheel(leftFront, leftFrontPower, max, min);
        controlWheel(rightFront, rightFrontPower, max, min);
        controlWheel(leftBack, leftBackPower, max, min);
        controlWheel(rightBack, rightBackPower, max, min);
    }

    private void controlWheel(DcMotor motor, double tgtPower, double maxPower, double minPower) {
        if (maxPower > forwardPowerLimit) {
            tgtPower *= forwardPowerLimit / maxPower;
        } else if (minPower > backwardPowerLimit) {
            tgtPower *= backwardPowerLimit / minPower;
        }

        // move motor
        motor.setPower(tgtPower);


        telemetry.addData(motor.getDeviceName(), tgtPower);
    }

    public void setNavigation() {
    }

}
