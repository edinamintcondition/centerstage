package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import data.Point;
import data.Position;

@Autonomous
public class AutoOpModeRedFront extends MintAutonomous {

    public AutoOpModeRedFront() {
        super(new Position(144 - frontStartX, frontStartY, 0, 1, 0));
    }

    public void driveToBackboard() {
        driveToClosestPoint(new Point(currentPos.x, frontStartY - 20));
        rotateToHeading(0);
        strafeToClosestPoint(new Point(144 - frontCentralX, frontStartY));
        rotateToHeading(0);
        driveToClosestPoint(new Point(144 - frontCentralX, approachY));
        rotateToHeading(0);
        strafeToClosestPoint(new Point(144 - backboardX, approachY));
        rotateToHeading(0);
    }

    public void park() {
        Point p = new Point(currentPos.x + parkX, currentPos.y - 6);
        driveToClosestPoint(p);
        strafeToClosestPoint(p);
    }
}
