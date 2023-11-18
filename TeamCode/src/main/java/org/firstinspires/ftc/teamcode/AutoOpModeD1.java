package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoOpModeD1 extends AutoOpMode {

    public AutoOpModeD1() {
        super (new Position(8.5, 36+24, 0, 1, 0));
    }

    public void driveToBackboard() {
        strafeToClosestPoint(new Point(11.75, 36+24));
        pause();
        rotateToHeading(0);
        pause();
        driveToClosestPoint(new Point(11.75, 96));
        pause();
        rotateToHeading(0);
        pause();
        strafeToClosestPoint(new Point(36, 96));
        pause();
        rotateToHeading(0);
        pause();
        driveToClosestPoint(new Point(36, 120.5));
        pause();
        rotateToHeading(0);
        pause();
        driveToClosestPoint(new Point(36, 120.5));
        pause();
        rotateToHeading(0);
        pause();
    }
}
