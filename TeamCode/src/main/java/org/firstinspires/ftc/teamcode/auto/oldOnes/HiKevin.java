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
import org.firstinspires.ftc.teamcode.subsystems.sensors.Limelight;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Autonomous(name="No Pattern Nine Ball - Hi Kevin")
@Config
@Disabled
public class HiKevin extends LinearOpMode {

    public List<String> order1 = new ArrayList<>(Arrays.asList("P", "P", "G"));

    public List<String> targetOrder = order1; // default

    public static double[] start = new double[] { -65, -41.75, 0};

    //Obelisk look
    public static double[] lookAtOb = new double[] {-22, -24, -214};


    //1st Spike!!
    public static double[] close1Shooting = new double[] {-24, -24, -135};
    public static double[] collect1Pre = new double[] { -12, -30, -90 };
    public static double[] collect1Mid = new double[] { -12, -22, -90 };
//    public static double[] collect1 = new double[] { -12, -39, -90 };
//    public static double[] collect2 = new double[] { -12, -44, -90 };
//    public static double[] collect3 = new double[] { -2, -49, -90 };

    public static double[] firstSpikeEnd = new double[] { -12, -52, -90 };
    public static double[] strafePos = new double[] { -36, -17, -90 };

    //2nd spike!!
    public static double[] collect2Pre = new double[] { 12, -28, -90 };

//    public static double[] collect4 = new double[] { 10, -40, -90 };
//    public static double[] collect5 = new double[] { 10, -45, -90 };
//    public static double[] collect6 = new double[] { 10, -50, -90 };

    public static double[] secondSpikeEnd = new double[] { 12, -52, -90 };
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;

    private static class PARAMS{
        private double COLLECT_DRIVE_MAX_POWER = 0.15;
    }
    public static HiKevin.PARAMS PARAMS = new HiKevin.PARAMS();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20); // faster updates

        robot = new BrainSTEMRobot(hardwareMap, telemetry, this, createPose(start));
        AutoActions.setRobot(robot);

        DrivePath driveToOb = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(lookAtOb)).setMaxLinearPower(1)
        );

        DrivePath driveToPreloadShoot = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting))
        );

        DrivePath driveToShootOne = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting))
        );

        DrivePath driveToShootTwo = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting))
        );

        //1st Spike ===================================================================

//        DrivePath driveToCollectFirstSpike = new DrivePath(robot.drive, telemetry,
//                new Waypoint(createPose(collect1Mid)).setSlowDownPercent(0.5),
//                new Waypoint(createPose(collect1Pre)).setSlowDownPercent(0.1),
//                new Waypoint(createPose(collect1)).setMaxLinearPower(0.1).setMaxTime(3)
//        );
        DrivePath driveToCollectFirstSpikeEnd = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect1Pre)),
                new Waypoint(createPose(firstSpikeEnd)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER)
        );

        DrivePath driveOffLine = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(strafePos))
        );

        //2nd Spike!! ===================================================================
//        DrivePath driveToCollect2Pre = new DrivePath(robot.drive, telemetry,
//                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(0.1)
//        );
        DrivePath driveToCollectSecondSpikeEnd = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2Pre)),
                new Waypoint(createPose(secondSpikeEnd)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER)
        );

//
//        DrivePath driveToCollectFourthSpike = new DrivePath(robot.drive, telemetry,
//                new Waypoint(createPose(collect4)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
//        );
//
//        DrivePath driveToCollectFifthSpike = new DrivePath(robot.drive, telemetry,
//                new Waypoint(createPose(collect5)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
//        );
//        DrivePath driveToCollectSixthSpike = new DrivePath(robot.drive, telemetry,
//                new Waypoint(createPose(collect6)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
//        );

        telemetry.addLine("AUTO IS DONE COMPILING");
        telemetry.update();
        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
//
//                                new ParallelAction(
//                                        AutoActions.shooterTurnOnClose(),
//                                        AutoActions.pivotClose(),
//
//                                        driveToPreloadShoot
//                                ),
//
//
//
////                                AutoActions.shootAll(),
//
//                                AutoActions.rampUp(),
////                            new SleepAction(0.2),
//                                  new SleepAction(0.2),
//                               AutoActions.moveSpindexer360(),
////                             new SleepAction(0.5),
//                                new SleepAction(0.4),
//
//                                AutoActions.rampDown(),
//                                AutoActions.turnShooterOnIdle(),
//                                new SleepAction(0.5),
//
//
//                                //1st Spike Does Work ==========================
//                                new ParallelAction(
//                                        AutoActions.setCollectorOn(),
//                                        driveToCollectFirstSpikeEnd
//                                ),
//
//
//
//                                new SleepAction(0.2),
//
//                                new ParallelAction(
//                                        AutoActions.setCollectorOff(),
//                                        AutoActions.pivotClose(),
//                                        AutoActions.shooterTurnOnClose()
//                                ),
//
//                                new ParallelAction(
//                                        driveToPreloadShoot
//                                ),
//
//                                AutoActions.rampUp(),
////                            new SleepAction(0.2),
//                                new SleepAction(0.2),
//                                AutoActions.moveSpindexer360(),
////                             new SleepAction(0.5),
//                                new SleepAction(0.5),
//
//                                AutoActions.rampDown(),
//                                AutoActions.turnShooterOnIdle(),
//                                new SleepAction(0.4),
//
//                                //2nd Spike ==========================

                                new ParallelAction(
                                        AutoActions.setCollectorOn(),
                                        driveToCollectSecondSpikeEnd
                                ),
                                driveToPreloadShoot

//                                new SleepAction(0.3),
//
//                               new ParallelAction(
//                                       AutoActions.shooterTurnOnClose(),
//                                       AutoActions.pivotClose(),
//                                       driveToPreloadShoot
//                               ),
//
//
//                                AutoActions.rampUp(),
////                            new SleepAction(0.2),
//                                new SleepAction(0.2),
//                                AutoActions.moveSpindexer360(),
////                             new SleepAction(0.5),
//                                new SleepAction(0.4),
//
//                                AutoActions.rampDown(),
//                                AutoActions.turnShooterOnIdle(),
//                                new SleepAction(.4),
//
//                                driveOffLine


                        ),
                        AutoActions.robotUpdate(telemetry)
                ));
    }

    public double motifRotation(int num) {

        String B1 = "";
        String B2 = "";
        String B3 = "";

        if (num == 1 ||num == 0) {
            B1 = "P";
            B2 = "P";
            B3 = "G";
        } else if ( num == 2 ){
            B1 = "P";
            B2 = "G";
            B3 = "P";
        }else if ( num == 3 ){
            B1 = "G";
            B2 = "P";
            B3 = "P";
        }
        List<String> order1 = new ArrayList<>(Arrays.asList(B3, B1, B2)); // TODO: CHECK


        List<String> order2 = new ArrayList<>(order1);
        Collections.rotate(order2, 1);

        List<String> order3 = new ArrayList<>(order2);
        Collections.rotate(order3, 1);

        if (order1.equals(targetOrder)) {
            return 0;
        } else if (order2.equals(targetOrder)) {
            return (double) 1024 /3;
        } else if  (order3.equals(targetOrder)){
            return (double) (2 * 1024) /3;
        }

        return 0;
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