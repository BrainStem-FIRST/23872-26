package org.firstinspires.ftc.teamcode.auto.oldOnes;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.UtilFunctions.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@Autonomous(name="Red Close")
@Deprecated
@Disabled
@Config
public class RedClose extends LinearOpMode {
    public static double[] start = new double[] { -62.5, 41, 0 };

    //1st Spike!!
    public static double[] close1Shooting = new double[] {-22, 22, 135};
    public static double[] collect1Pre = new double[] { -12, 30, 90 };
    public static double[] collect1Mid = new double[] { -12, 22, 90 };
    public static double[] collect1 = new double[] { -12, 39, 90 };
    public static double[] collect2 = new double[] { -12, 44, 90 };
    public static double[] collect3 = new double[] { -12, 49, 90 };
    public static double[] strafePos = new double[] { -36, 17, 90 };

    //2nd spike!!
    public static double[] collect2Pre = new double[] { 10, 28, 90 };
    public static double[] collect2Mid = new double[] { 10, 22, 90 };

    public static double[] collect4 = new double[] { 10, 39, 90 };
    public static double[] collect5 = new double[] { 10, 43, 90 };
    public static double[] collect6 = new double[] { 10, 39, 90 };
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;

    private static class PARAMS{

        // Comes after collect a spike and before spindexing
        private double COLLECT_TO_SPIND_WAIT = 0.8;
        // Comes after spindexer and before drive
        private double SPIND_TO_DRIVE_WAIT = 0.3;

        // Max power for collecting artifacts
        private double COLLECT_DRIVE_MAX_POWER = 0.15;
    }
    public static RedClose.PARAMS PARAMS = new RedClose.PARAMS();

    public SequentialAction ShootingSequence() {
        return new SequentialAction(
                new SleepAction(0.4),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.2),
                new SleepAction(0.4),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.2),
                new SleepAction(0.4),
                AutoActions.moveSpindexer60(),
                AutoActions.turnShooterOnIdle()
        );
    }

    public SequentialAction CollectingSequence() {
        return new SequentialAction(
                new SleepAction(0.5),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.5),
                AutoActions.moveSpindexer120()
        );
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20); // faster updates

        robot = new BrainSTEMRobot(hardwareMap, telemetry, this, createPose(start));
        AutoActions.setRobot(robot);


        DrivePath driveToPreloadShoot = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting)).setMaxLinearPower(1)
        );

        //1st Spike ===================================================================
        DrivePath driveToCollect1Pre = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect1Mid)).setSlowDownPercent(0.5),
                new Waypoint(createPose(collect1Pre)).setSlowDownPercent(0.1)
        );
        DrivePath driveToCollectFirstSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect1)).setMaxLinearPower(0.1).setMaxTime(3)
        );

        DrivePath driveToCollectSecondSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2)).setMaxLinearPower(0.1).setMaxTime(3)
        );
        DrivePath driveToCollectThirdSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect3)).setMaxLinearPower(0.125).setMaxTime(3)
        );
        DrivePath driveOffLine = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(strafePos)).setMaxLinearPower(0.5)
        );

        //2nd Spike!! ===================================================================
        DrivePath driveToCollect2Pre = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(0.3),
                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(0.1)
        );
        DrivePath driveToCollectFourthSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect4)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );

        DrivePath driveToCollectFifthSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect5)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );
        DrivePath driveToCollectSixthSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect6)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // Ramp up shooter and drive to preload shoot
                                AutoActions.shooterTurnOnClose(),
                                new ParallelAction(
                                        AutoActions.waitForAccurateShooterVelocity(),
                                        driveToPreloadShoot
                                ),

                                // "Domino Sequence"
                                ShootingSequence(),

                                AutoActions.setCollectorOn(),
                                new SleepAction(0.3),

                                //1st Spike Does Work ==========================
                                driveToCollect1Pre,
                                driveToCollectFirstSpike,
//                                AutoActions.setCollectorOff(),
                                new SleepAction(0.6),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                driveToCollectSecondSpike,
                                new SleepAction(PARAMS.COLLECT_TO_SPIND_WAIT),

                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                driveToCollectThirdSpike,
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),

                                new ParallelAction(
                                        AutoActions.setCollectorOff(),
                                        AutoActions.moveSpindexer60()
                                ),


                                // Shooting sequence for 1st spike
                                // Turn shooter back on
                                AutoActions.shooterTurnOnClose(),
                                // Drive to shoot position
                                driveToPreloadShoot,
                                new SleepAction(1.1),
                                // Last shooting sequence

                                ShootingSequence(),
                                new SleepAction(0.3),

                                //2nd Spike May Not Work ==========================
                                AutoActions.setCollectorOn(),
                                new SleepAction(0.3),
                                driveToCollect2Pre,
                                driveToCollectFourthSpike,
                                new SleepAction(0.6),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                driveToCollectFifthSpike,
                                new SleepAction(PARAMS.COLLECT_TO_SPIND_WAIT),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                driveToCollectSixthSpike,
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                AutoActions.moveSpindexer60(),
                                new SleepAction(0.3),
                                driveToPreloadShoot,
                                new SleepAction(0.3),
                                ShootingSequence(),
                                driveOffLine


                        ),
                        AutoActions.robotUpdate(telemetry)
                ));
    }



}