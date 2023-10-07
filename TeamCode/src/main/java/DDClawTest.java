import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import parts.MintMotor;
import parts.MintServo;

@TeleOp
public class DDClawTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        MintServo mintServo = new MintServo(hardwareMap);
        MintMotor mintMotor = new MintMotor(hardwareMap);

        String button = "heppe";
        waitForStart();

        telemetry.addData(">", "Starting Program");
        mintServo.printPosition(telemetry);
        telemetry.update();

        while (opModeIsActive()) {
            mintMotor.control(gamepad2);

            button = mintServo.control(gamepad2);

            telemetry.addData(">", button + " is pressed :D");
            mintServo.printPosition(telemetry);
            telemetry.update();
        }
    }
}



