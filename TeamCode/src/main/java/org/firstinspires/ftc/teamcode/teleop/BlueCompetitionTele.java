package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BLUE Competition Tele Yay")
public class BlueCompetitionTele extends CompetitionTele {
    @Override
    public void runOpMode() throws InterruptedException {
        red = false;
        super.runOpMode();
    }
}
