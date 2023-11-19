package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoOpModeBlueBack extends AutoOpMode {
    public AutoOpModeBlueBack() {
        super(new Position(backStartX, backStartY, 0, 1, 0));
    }

    public void driveToBackboard() {
        strafeToClosestPoint(new Point(backboardX, backStartY));
        rotateToHeading(0);
        driveToClosestPoint(new Point(backboardX, approachY));
        rotateToHeading(0);
    }

    public void park() {
        Point p = new Point(currentPos.x - 12, currentPos.y - 6);
        driveToClosestPoint(p);
        strafeToClosestPoint(p);
    }
}
