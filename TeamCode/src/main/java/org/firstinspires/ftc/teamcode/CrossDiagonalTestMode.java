package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import parts.DriveDirection;

@Autonomous
public class CrossDiagonalTestMode extends DriveTestMode {
    public CrossDiagonalTestMode() {
        dir = DriveDirection.CrossDiagonal;
    }
}
