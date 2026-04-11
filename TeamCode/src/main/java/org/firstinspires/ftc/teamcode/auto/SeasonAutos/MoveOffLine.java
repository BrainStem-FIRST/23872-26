package org.firstinspires.ftc.teamcode.auto.SeasonAutos;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.UtilFunctions.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.auto.AutoActions;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="Move Off Line Blue", group = "BLUE")
@Config

public class MoveOffLine extends LinearOpMode {
    public List<String> order1 = new ArrayList<>(Arrays.asList("P", "P", "G"));

    public List<String> targetOrder = order1; // default


    public static double[] start = new double[] { 65, -30.25, -90};

    //Obelisk look
    public static double[] lookAtOb = new double[] {-23,-23, -195};
    public static double[] driveOff = new double[] { 65, -35.25, -90};

    //Open Gate

    BrainSTEMRobot robot;
    private static class PARAMS{
        private double COLLECT_DRIVE_MAX_POWER = 0.15;
    }
    public static MoveOffLine.PARAMS PARAMS = new MoveOffLine.PARAMS();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20); // faster updates

        robot = new BrainSTEMRobot(hardwareMap, telemetry, this, createPose(start));
        AutoActions.setRobot(robot);

        DrivePath drivingOff = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(driveOff)).setMaxLinearPower(0.5).setMaxTime(1.5)
        );



        telemetry.addLine("AUTO IS DONE COMPILING");
        telemetry.update();
        waitForStart();

        Action autoAction = new ParallelAction(
                new SequentialAction(
                      drivingOff



                ),
                    AutoActions.robotUpdate(telemetry)
        );

        Actions.runBlocking(autoAction);
    }

    public boolean updateTargetMotif() {
        int tagId = Limelight.feducialResult;

        switch (tagId) {
            case 21:
                targetOrder = new ArrayList<>(Arrays.asList("G", "P", "P"));
                break;
            case 22:
                targetOrder = new ArrayList<>(Arrays.asList("P", "G", "P"));
                break;
            case 23:
                targetOrder = new ArrayList<>(Arrays.asList("P", "P", "G"));
                break;
            default:
                break;
        }

        return true;
    }
}