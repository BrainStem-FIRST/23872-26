package org.firstinspires.ftc.teamcode.auto.SeasonAutos;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.UtilFunctions.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.auto.AutoActions;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="1 Gate Close - Red 12", group = "RED")
@Config

public class OneGRedClose12 extends LinearOpMode {
    public List<String> order1 = new ArrayList<>(Arrays.asList("P", "P", "G"));

    public List<String> targetOrder = order1; // default


    public static double[] start = new double[] { -65, 41.75, 0};

    //Obelisk look
    public static double[] lookAtOb = new double[] {-22,22, 195};

    //Open Gate
    public static double[] openGatePos = new double[] {-7,72-6-5.25, -180};
    public static double[] limelight = new double[] {-24, 24, -135};



    //1st Spike!!
    public static double[] close1Shooting = new double[] {-39, 39, 135};
    public static double[] collect1Pre = new double[] { -13, 29, 90 };
    public static double[] collect1Mid = new double[] { -13, 22, 90 };
//    public static double[] collect1 = new double[] { -12, -39, -90 };
//    public static double[] collect2 = new double[] { -12, -44, -90 };
//    public static double[] collect3 = new double[] { -2, -49, -90 };

    public static double[] collect2Mid = new double[] { 7, 48, 90 };

    public static double[] collect3Pre = new double[] { 33, 22, 90 };
    public static double[] collect3PrePass = new double[] { 10, 48, 90 };


    public static double[] thirdSpikeENd = new double[] { 33, 50, 90 };

    public static double[] firstSpikeEnd = new double[] { -12, 51, 90 };
    public static double[] strafePos = new double[] { -17, 36, 90 };

    //2nd spike!!
    public static double[] collect2Pre = new double[] { 10, 22, 90 };


//    public static double[] collect4 = new double[] { 10, -40, -90 };
//    public static double[] collect5 = new double[] { 10, -45, -90 };
//    public static double[] collect6 = new double[] { 10, -50, -90 };

    public static double[] secondSpikeEnd = new double[] { 10, 51, 90 };
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;
    private static class PARAMS{
        private double COLLECT_DRIVE_MAX_POWER = 0.23;
    }
    public static OneGRedClose12.PARAMS PARAMS = new OneGRedClose12.PARAMS();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20); // faster updates

        robot = new BrainSTEMRobot(hardwareMap, telemetry, this, createPose(start));
        AutoActions.setRobot(robot);




        DrivePath openGate = new DrivePath(robot.drive, telemetry,

                new Waypoint(createPose(openGatePos)).setMaxLinearPower(1).setMaxTime(1),
                new Waypoint(createPose(limelight))
        );

        DrivePath driveToCollectThirdSpikeEnd = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect3Pre)),
                new Waypoint(createPose(thirdSpikeENd)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER)
        );

        DrivePath driveToPreloadShoot = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting))
        );

        DrivePath driveToShootOne = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting))
        );

        DrivePath driveToShootTwo = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2Mid),new Tolerance(2.5, 2.5, 6)).setMinLinearPower(0.3).setMaxTime(1),
                new Waypoint(createPose(close1Shooting))
        );
        DrivePath driveToShoot3 = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect3PrePass), new Tolerance(2.5, 2.5, 6)).setMinLinearPower(0.3).setMaxTime(1),
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
                new Waypoint(createPose(firstSpikeEnd)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(4)
        );

        DrivePath driveOffLine = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(strafePos))
        );

        //2nd Spike!! ===================================================================
//        DrivePath driveToCollect2Pre = new DrivePath(robot.drive, telemetry,
//                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(0.1)
//        );
        DrivePath driveToCollectSecondSpikeEnd = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2Pre)).setMinLinearPower(0.3),
                new Waypoint(createPose(secondSpikeEnd)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(4)
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

        Action autoAction = new ParallelAction(
                new SequentialAction(
                        new ParallelAction(
                                AutoActions.shooterTurnOnClose(),
                                driveToPreloadShoot
                        ),

                        new SleepAction(1),



                        AutoActions.rampUp(),
                        new SleepAction(0.2),


                        AutoActions.moveSpindexer360(),
                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle(),


                        // skips to this:


                        // GATE

                        new ParallelAction(
                                AutoActions.setCollectorOn(),
                                driveToCollectFirstSpikeEnd
                        ),


                        new ParallelAction(
                                openGate,
                                AutoActions.setCollectorOff(),
                                AutoActions.pivotClose(),
                                AutoActions.shooterTurnOnClose()
                        ),



                        AutoActions.waitForLimelightAuto(),


                        new ParallelAction(
                                driveToShootOne,
                                AutoActions.moveSpindexerMot(1, telemetry)
                        ),



                        AutoActions.rampUp(),
//                            new SleepAction(0.2),
                        new SleepAction(0.2),
                        AutoActions.moveSpindexer360(),

                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle(),

                        //2nd Spike ==========================

                        new ParallelAction(
                                AutoActions.setCollectorOn(),
                                driveToCollectSecondSpikeEnd
                        ),


                        new ParallelAction(
                                AutoActions.setCollectorOff(),
                                AutoActions.shooterTurnOnClose()
                                , AutoActions.pivotClose()
                                , driveToShootTwo,

                                AutoActions.moveSpindexerMot(2, telemetry)
                        ),



                        AutoActions.rampUp(),
//                            new SleepAction(0.2),
                        new SleepAction(0.2),
                        AutoActions.moveSpindexer360(),
                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle(),
                        //3rd

                        new ParallelAction(
                                AutoActions.setCollectorOn(),
                                driveToCollectThirdSpikeEnd
                        ),

                        new ParallelAction(
                                AutoActions.setCollectorOff(),
                                AutoActions.shooterTurnOnClose()
                                , AutoActions.pivotClose()
                                , driveToShoot3
                        ),


                        AutoActions.moveSpindexerMot(3, telemetry),

                        AutoActions.rampUp(),
                        new SleepAction(0.2),
                        AutoActions.moveSpindexer360(),

                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle(),

                        driveOffLine




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