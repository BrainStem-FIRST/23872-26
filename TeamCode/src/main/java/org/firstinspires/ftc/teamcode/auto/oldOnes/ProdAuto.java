package org.firstinspires.ftc.teamcode.auto.oldOnes;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.UtilFunctions.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.auto.AutoActions;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;
@Disabled
@Deprecated
@Autonomous(name="untested Blue Close")
@Config
public class ProdAuto extends LinearOpMode {
    public static double[] start = new double[] { -62.5, -41, 0 };

    // 1st Spike
    public static double[] close1Shooting = new double[] {-24, -24, -135};
    public static double[] collect1Pre = new double[] { -13, -30, -90 };
    public static double[] collect1Mid = new double[] { -13, -22, -90 };
    public static double[] collect1 = new double[] { -13, -39, -90 };
    public static double[] collect2 = new double[] { -13, -44, -90 };
    public static double[] collect3 = new double[] { -13, -49, -90 };
    public static double[] strafePos = new double[] { -36, -17, -90 };

    // 2nd spike
    public static double[] collect2Pre = new double[] { 11, -30, -90 };
    public static double[] collect4 = new double[] { 11, -39, -90 };
    public static double[] collect5 = new double[] { 11, -44, -90 };
    public static double[] collect6 = new double[] { 11, -49, -90 };

    // 3rd spike
    public static double[] collect3Pre = new double[] { 36, -30, -90 };
    public static double[] collect3Mid = new double[] { 36, -21, -90 };
    public static double[] collect7 = new double[] { 36, -39, -90 };
    public static double[] collect8 = new double[] { 36, -44, -90 };
    public static double[] collect9 = new double[] { 36, -49, -90 };
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;

    private static class PARAMS{
        private double COLLECT_DRIVE_MAX_POWER = 0.3;
    }
    public static ProdAuto.PARAMS PARAMS = new ProdAuto.PARAMS();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20); // faster updates

        robot = new BrainSTEMRobot(hardwareMap, telemetry, this, createPose(start));
        AutoActions.setRobot(robot);


        // WAYPOINTS ===============================================================================


        DrivePath driveToPreloadShoot = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting)).setMaxLinearPower(1)
        );

        // 1st Spike -------

        DrivePath driveToCollectFirstSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect1Pre)).setSlowDownPercent(0.2),
                new Waypoint(createPose(collect3)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );


        // 2nd Spike -------

        DrivePath driveToCollectSecondSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(0.2),
                new Waypoint(createPose(collect6)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );

        // 3rd Spike -------
        DrivePath driveToCollectThirdSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(0.2),
                new Waypoint(createPose(collect9)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );

        // Drive off line -------
        DrivePath driveOffLine = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(strafePos))
        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                AutoActions.pivotClose(),
                                // PRELOAD SHOOT
                                new ParallelAction(
                                        AutoActions.shooterTurnOnClose(),
                                        driveToPreloadShoot,
                                        AutoActions.waitForAccurateShooterVelocity()
                                ),
                                AutoActions.shootAll(),

                                // 1ST SPIKE

                                new ParallelAction(
                                        AutoActions.setCollectorOn(),
                                        driveToCollectFirstSpike
                                ),

                                // trans to shoot
                                new ParallelAction(
                                        AutoActions.setCollectorOff(),
                                        AutoActions.shooterTurnOnClose(),
                                        AutoActions.waitForAccurateShooterVelocity(),
                                        driveToPreloadShoot
                                ),

                                AutoActions.shootAll(),

                                // 2ND SPIKE

                                new ParallelAction(
                                        AutoActions.setCollectorOn(),
                                        driveToCollectSecondSpike

                                ),


                                // trans to shoot
                                new ParallelAction(
                                        AutoActions.setCollectorOff(),
                                        AutoActions.shooterTurnOnClose(),
                                        AutoActions.waitForAccurateShooterVelocity(),
                                        driveToPreloadShoot
                                ),

                                AutoActions.shootAll(),

                                driveOffLine


                        ),
                        AutoActions.robotUpdate(telemetry)
                ));
    }



}