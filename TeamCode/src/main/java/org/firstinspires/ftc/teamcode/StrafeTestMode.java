package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import parts.DriveDirection;

@Autonomous
public class StrafeTestMode extends DriveTestMode{
    public StrafeTestMode() {
        dir = DriveDirection.Lateral;
    }
}
