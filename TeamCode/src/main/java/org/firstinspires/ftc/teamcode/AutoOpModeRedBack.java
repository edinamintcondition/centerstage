package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoOpModeRedBack extends MintAutonomous {
    public AutoOpModeRedBack() {
        super(new Position(144 - backStartX, backStartY, 0, 1, 0));
    }

    public void driveToBackboard() {
        strafeToClosestPoint(new Point(144 - backboardX, backStartY));
        rotateToHeading(0);
        driveToClosestPoint(new Point(144 - backboardX, approachY));
        rotateToHeading(0);
    }
    public void park() {
        Point p = new Point(currentPos.x + 12, currentPos.y - 6);
        driveToClosestPoint(p);
        strafeToClosestPoint(p);
    }
}
