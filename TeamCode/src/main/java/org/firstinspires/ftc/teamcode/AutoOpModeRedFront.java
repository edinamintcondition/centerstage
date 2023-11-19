package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoOpModeRedFront extends AutoOpMode {

    public AutoOpModeRedFront() {
        super(new Position(8.5, 36, 0, 1, 0));
    }

    public void driveToBackboard() {
        strafeToClosestPoint(new Point(11.75, 36));
        rotateToHeading(0);
        driveToClosestPoint(new Point(11.75, 96));
        rotateToHeading(0);
        strafeToClosestPoint(new Point(36, 96));
        rotateToHeading(0);
        driveToClosestPoint(new Point(36, 120.5));
        rotateToHeading(0);
        driveToClosestPoint(new Point(36, 120.5));
        rotateToHeading(0);
    }
}
