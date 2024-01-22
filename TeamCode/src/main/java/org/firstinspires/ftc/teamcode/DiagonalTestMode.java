package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import parts.DriveDirection;

@Autonomous
public class DiagonalTestMode extends DriveTestMode {
    public DiagonalTestMode() {
        dir = DriveDirection.Diagonal;
    }
}
