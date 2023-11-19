package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoOpModeRedBack extends AutoOpMode {
    public AutoOpModeRedBack() {
        super(new Position(144 - 8.5, 84, 0, 1, 0));
    }

    public void driveToBackboard() {
        strafeToClosestPoint(new Point(144 - 36, 84));
        rotateToHeading(0);
        driveToClosestPoint(new Point(144 - 36, 114));
        rotateToHeading(0);
        driveToClosestPoint(new Point(144 - 36, 120.5));
        rotateToHeading(0);
    }
}
