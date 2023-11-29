package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import parts.MintAutonomous;
import parts.MintWrist;
import parts.MintGrabber;
import parts.MintArm;
import parts.MintCamera;

@Autonomous
public class AutoOpModeBlueFront extends MintAutonomous {

    public AutoOpModeBlueFront() {
        super(new Position(frontStartX, frontStartY, 0, 1, 0));
    }

    public void driveToBackboard() {
        strafeToClosestPoint(new Point(frontCentralX, frontStartY));
        rotateToHeading(0);
        driveToClosestPoint(new Point(frontCentralX, approachY));
        rotateToHeading(0);
        strafeToClosestPoint(new Point(backboardX, approachY));
        rotateToHeading(0);
    }

    public void park() {
        Point p = new Point(currentPos.x - 12, currentPos.y - 6);
        driveToClosestPoint(p);
        strafeToClosestPoint(p);
    }
}