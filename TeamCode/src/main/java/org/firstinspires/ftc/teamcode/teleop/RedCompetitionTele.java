package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RED Competition Tele Yay")
public class RedCompetitionTele extends CompetitionTele {
    @Override
    public void runOpMode() throws InterruptedException {
        red = true;
        super.runOpMode();
    }
}
