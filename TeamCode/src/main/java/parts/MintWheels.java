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
        rightFront.setDirection(REVERSE);

        leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        leftBack.setDirection(FORWARD);

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

        double powerLimit = 0.6;
        if (gamepad.right_bumper) {
            powerLimit = 1;
            telemetry.addData(">", "TURBO LETS GOOOO");
        } else if (gamepad.left_bumper) {
            powerLimit = 0.3;
            telemetry.addData( ">", "slow speed :b");
        }

        runAny(axial, lateral, yaw, powerLimit);
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

    public void runAny(double axial, double lateral, double yaw, double powerLimit) {
        double max;

        double leftFrontPower = axial + lateral - yaw;
        double leftBackPower = axial - lateral - yaw;
        double rightFrontPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral + yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > powerLimit) {
            leftFrontPower *= powerLimit / max;
            rightFrontPower *= powerLimit / max;
            leftBackPower *= powerLimit / max;
            rightBackPower *= powerLimit / max;
        }

        //Applies the power to the motors
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }
}