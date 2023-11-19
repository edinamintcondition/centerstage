package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoOpModeBlueBack extends AutoOpMode {
    public AutoOpModeBlueBack() {
        super(new Position(8.5, 84, 0, 1, 0));
    }

    public void driveToBackboard() {
        strafeToClosestPoint(new Point(36, 84));
        rotateToHeading(0);
        driveToClosestPoint(new Point(36, 114));
        rotateToHeading(0);
        driveToClosestPoint(new Point(36, 120.5));
        rotateToHeading(0);
    }
}
