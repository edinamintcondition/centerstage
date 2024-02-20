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

        //Calculates the wheel direction
        double leftFrontPower = axial + lateral + (yaw * 5);
        double leftBackPower = axial - lateral + (yaw * 5);
        double rightFrontPower = -axial + lateral + (yaw * 5);
        double rightBackPower = axial + lateral - (yaw * 5);

        double max = max(asList(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower));
        double min = min(asList(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower));


        if (gamepad.right_bumper) {
            forwardPowerLimit = 1.0;
            backwardPowerLimit = -1;
            telemetry.addData(">", "TURBO LETS GOOOO");
        } else if (gamepad.left_bumper) {
            forwardPowerLimit = 0.2;
            backwardPowerLimit = -0.2;
            telemetry.addData(">", "SLOOOW DOWN MATE");
        } else {
            forwardPowerLimit = 0.45;
            backwardPowerLimit = -0.45;
            telemetry.addData(">", "normal speed :b");
        }

        controlWheel("Left Front", leftFront, leftFrontPower, max, min);
        controlWheel("Right Front", rightFront, rightFrontPower, max, min);
        controlWheel("Left Back", leftBack, leftBackPower, max, min);
        controlWheel("Right Back", rightBack, rightBackPower, max, min);
    }

    private void controlWheel(String name, DcMotor motor, double tgtPower, double maxPower, double minPower) {
        if (maxPower > forwardPowerLimit) {
            tgtPower *= forwardPowerLimit / maxPower;
        } else if (minPower < backwardPowerLimit) {
            tgtPower *= backwardPowerLimit / minPower;
        }

        // move motor
        motor.setPower(tgtPower);


        telemetry.addData(">", name + " motor: ", tgtPower);
    }


//    public void runAny(double axial, double lateral, double yaw, double powerLimit) {
//        double max;
//
//        double leftFrontPower = axial + lateral - yaw;
//        double leftBackPower = axial - lateral - yaw;
//        double rightFrontPower = axial - lateral + yaw;
//        double rightBackPower = axial + lateral + yaw;
//
//        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > powerLimit) {
//            leftFrontPower *= powerLimit / max;
//            rightFrontPower *= powerLimit / max;
//            leftBackPower *= powerLimit / max;
//            rightBackPower *= powerLimit / max;
//        }
//
//        //Applies the power to the motors
////        leftFront.setPower(leftFrontPower);
////        leftBack.setPower(leftBackPower);
//        rightFront.setPower(rightFrontPower);
////        rightBack.setPower(rightBackPower);
//    }
}
