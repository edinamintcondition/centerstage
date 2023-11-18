package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoOpModeB6 extends AutoOpMode {

    public AutoOpModeB6() {
        super (new Position(144-8.5, 36, 0, 1, 0));
    }

    public void driveToBackboard() {
        strafeToClosestPoint(new Point(144-11.75, 36));
        pause();
        rotateToHeading(0);
        pause();
        driveToClosestPoint(new Point(144-11.75, 96));
        pause();
        rotateToHeading(0);
        pause();
        strafeToClosestPoint(new Point(144-36, 96));
        pause();
        rotateToHeading(0);
        pause();
        driveToClosestPoint(new Point(144-36, 120.5));
        pause();
        rotateToHeading(0);
        pause();
        driveToClosestPoint(new Point(144-36, 120.5));
        pause();
        rotateToHeading(0);
        pause();
    }
}
