package org.firstinspires.ftc.teamcode.auto.helping_opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name = "Spind Test")
@Disabled
public class SpindexerTest extends OpMode {

    private BrainSTEMRobot robot;
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(BrainSTEMRobot.autoX, BrainSTEMRobot.autoY, BrainSTEMRobot.autoH));

    }

    public void runOpMode() throws InterruptedException {
        if (gamepad1.a) {
            robot.spindexer.SPINDEXER_TIME = 0;
            robot.spindexer.setTargetAdj(Constants.spindexerConstants.TICKS_120);
        }

    }

    public void loop() {

        telemetry.addData("Spind ticks", robot.spindexer.spindexerMotor.getCurrentPosition());
        telemetry.update();


    }
}