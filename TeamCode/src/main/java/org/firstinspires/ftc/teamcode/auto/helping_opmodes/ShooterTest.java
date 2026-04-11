package org.firstinspires.ftc.teamcode.auto.helping_opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;

@TeleOp(name = "Spind Test")
@Disabled
public class ShooterTest extends OpMode {

    private BrainSTEMRobot robot;
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(BrainSTEMRobot.autoX, BrainSTEMRobot.autoY, BrainSTEMRobot.autoH));

    }

    public void runOpMode() throws InterruptedException {

        if (gamepad1.a) {

            robot.shooter.shooterMotorOne.setPower(0.5);
        }

        if (gamepad1.b) {

            robot.shooter.shooterMotorTwo.setPower(0.5);
        }

    }

    public void loop() {

        telemetry.addData("Spind ticks", robot.spindexer.spindexerMotor.getCurrentPosition());
        telemetry.update();


    }
}