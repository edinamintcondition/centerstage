package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import data.Point;
import data.Position;

@Autonomous
public class AutoOpModeRedBack extends MintAutonomous {
    public AutoOpModeRedBack() {
        super(new Position(144 - backStartX, backStartY, 0, 1, 0));
    }

    public void driveToBackboard() {
        driveToClosestPoint(new Point(144 - backboardX, approachY));
        rotateToHeading(0);
        strafeToClosestPoint(new Point(144 - backboardX, backStartY));
        rotateToHeading(0);

    }
    public void park() {
        Point p = new Point(currentPos.x + parkX, currentPos.y - 6);
        driveToClosestPoint(p);
        strafeToClosestPoint(p);
    }
}