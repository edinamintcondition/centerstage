package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import data.Point;
import data.Position;

@Autonomous
public class AutoOpModeBlueBack extends MintAutonomous {
    public AutoOpModeBlueBack() {
        super(new Position(backStartX, backStartY, 0, 1, 0));
    }

    public void driveToBackboard() {
        driveToClosestPoint(new Point(backboardX, approachY));
        rotateToHeading(0);
        strafeToClosestPoint(new Point(backboardX, backStartY));
        rotateToHeading(0);
    }

    public void park() {
        Point p = new Point(currentPos.x - parkX, currentPos.y - 6);
        driveToClosestPoint(p);
        strafeToClosestPoint(p);
    }
}
