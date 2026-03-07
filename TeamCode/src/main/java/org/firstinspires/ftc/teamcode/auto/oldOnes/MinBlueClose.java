package org.firstinspires.ftc.teamcode.auto.oldOnes;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.UtilFunctions.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.auto.AutoActions;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@Autonomous(name="untested Blue Close")
@Config
@Disabled
@Deprecated
public class MinBlueClose extends LinearOpMode {
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

        // Comes after collect a spike and before spindexing
        private double COLLECT_TO_SPIND_WAIT = 0.8;
        // Comes after spindexer and before drive
        private double SPIND_TO_DRIVE_WAIT = 0.3;

        // Max power for collecting artifacts
        private double COLLECT_DRIVE_MAX_POWER = 0.3;
    }
    public static MinBlueClose.PARAMS PARAMS = new MinBlueClose.PARAMS();

    public SequentialAction ShootingSequence() {
        return new SequentialAction(
                new SleepAction(0.4),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.3),
                new SleepAction(0.4),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.3),
                new SleepAction(0.4),
                AutoActions.moveSpindexer60(),
                AutoActions.turnShooterOnIdle()
        );
    }

    public Action triggerSpindexerAtPos(double targetY) {
        return new Action() {
            private boolean triggered = false;
            @Override
            public boolean run(TelemetryPacket packet) {
                if (!triggered && Math.abs(robot.drive.localizer.getPose().position.y - targetY) < 2.0) {
                    robot.spindexer.setTargetAdj(80);
                    triggered = true;
                }
                return !triggered;
            }
        };
    }

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
                                // PRELOAD SHOOT
                                new ParallelAction(
                                        AutoActions.shooterTurnOnClose(),
                                        driveToPreloadShoot,
                                        AutoActions.waitForAccurateShooterVelocity()
                                ),
                                ShootingSequence(),

                                // 1ST SPIKE

                                new ParallelAction(
                                        AutoActions.setCollectorOn(),
                                        driveToCollectFirstSpike,
                                        new SequentialAction(
                                                triggerSpindexerAtPos(-44),
                                                triggerSpindexerAtPos(-49)
                                        )
                                ),

                                // trans to shoot
                                new ParallelAction(
                                        AutoActions.setCollectorOff(),
                                        AutoActions.moveSpindexer60(),
                                        AutoActions.shooterTurnOnClose(),
                                        AutoActions.waitForAccurateShooterVelocity(),
                                        driveToPreloadShoot
                                ),

                                ShootingSequence(),

                                // 2ND SPIKE

                                new ParallelAction(
                                        AutoActions.setCollectorOn(),
                                        driveToCollectSecondSpike,
                                        new SequentialAction(
                                                triggerSpindexerAtPos(-44),
                                                triggerSpindexerAtPos(-49)
                                        )
                                ),


                                // trans to shoot
                                new ParallelAction(
                                        AutoActions.moveSpindexer60(),
                                        AutoActions.setCollectorOff(),
                                        AutoActions.shooterTurnOnClose(),
                                        AutoActions.waitForAccurateShooterVelocity(),
                                        driveToPreloadShoot
                                ),

                                ShootingSequence(),

                                driveOffLine


                        ),
                        AutoActions.robotUpdate(telemetry)
                ));
    }



}