package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoOpModeRedBack extends AutoOpMode {

    public AutoOpModeRedBack() {
        super(new Position(144 - 8.5, 84, 0, 1, 0));
    }

    public void driveToBackboard() {
        strafeToClosestPoint(new Point(144 - 11.75, 84));
        rotateToHeading(0);
        driveToClosestPoint(new Point(144 - 11.75, 96));
        rotateToHeading(0);
        strafeToClosestPoint(new Point(144 - 36, 96));
        rotateToHeading(0);
        driveToClosestPoint(new Point(144 - 36, 120.5));
        rotateToHeading(0);
        driveToClosestPoint(new Point(144 - 36, 120.5));
        rotateToHeading(0);
    }
}
