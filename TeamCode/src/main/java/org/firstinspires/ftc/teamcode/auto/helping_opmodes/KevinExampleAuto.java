package org.firstinspires.ftc.teamcode.auto.helping_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;


@Config
@Disabled
@Autonomous(name="kevin example auto")
public class KevinExampleAuto extends LinearOpMode {
    public static double x = 48, y = 0, h = 0;
    public static double x2 = 0, y2 = 0, h2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(0, 0, 0);
        Pose2d p1 = new Pose2d(x, y, h);
        Pose2d p2 = new Pose2d(x2, y2, h2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this, start);
        DrivePath path = new DrivePath(robot.drive, telemetry,
                new Waypoint(p1)
        );
        // READ THIS INFO VERY CAREFULLY
        // the robot starts at "start" and will drive to "end" and then drive back to "start"
        // why will it do this? because when I declare the "path" variable, I pass in TWO Waypoint objects
        // Whenever you declare a waypoint, you need to pass in the pose of the waypoint
        // the drivetrain will follow the waypoints in the order you declare them
        // if you want to create another drive path then just do:
        // DrivePath path2 = new DrivePath(robot.drive, etc pass in your waypoints here...)

        // for right now, there are 5 values you need to tune
        // 2 of them are located in PathParams (inside this file you will see a class called DefaultParams; its in this inner class)
        //   farHeadingKp: if you increase it, the robot will rotate faster but will oscillate more (vice versa)
        //   speedKp: if you increase it, the robot will drive faster but will oscillate more (and vice versa)
        // the other 3 values you need to tune are in Tolerance class
        //   xTol, yTol, and headingTol
        // the robot won't end the path until the position and heading errors are within these tolerances (so I suggest setting them to something like 3, 3, and Math.toRadians(5))

        // how do you know when you're done tuning? idk you decide, when the robot is not oscillating at each waypoint anymore
        waitForStart();
//        Actions.runBlocking(robot.drive.actionBuilder(start).strafeToLinearHeading(p1.position, p1.heading.toDouble()).build());
        Actions.runBlocking(path);
//        Actions.runBlocking(packet -> {
//            robot.drive.updatePoseEstimate();
//            Pose2d robotPose = robot.drive.localizer.getPose();
//            Drawing.drawRobot(packet.fieldOverlay(), robotPose);
//            return true;
//        });
    }
}
