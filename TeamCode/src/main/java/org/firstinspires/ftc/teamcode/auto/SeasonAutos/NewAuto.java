package org.firstinspires.ftc.teamcode.auto.SeasonAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.auto.AutoActions;

@Autonomous(name="YOYOYOYOYOY", group = "YOYOYOY")
public class NewAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this, new Pose2d(0, 0, 0));
        AutoActions.setRobot(robot);

        Runnable turnOnIntake = new Runnable() {
            @Override
            public void run() {
                robot.collector.on();
            }
        };

        telemetry.addLine("READYYYY");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            boolean seesBall = robot.limelight.getClosestBall() != null;

            telemetry.addData("Ball Detected", seesBall);
            telemetry.update();
        }

        if (isStopRequested()) return;



        Actions.runBlocking(
            new ParallelAction(
                    robot.limelight.fullCollect(() -> robot.collector.on()),
                    AutoActions.robotUpdate(telemetry)
            )
        );



        telemetry.addLine("DONEEE");
        telemetry.update();
        sleep(2000);
    }
}
