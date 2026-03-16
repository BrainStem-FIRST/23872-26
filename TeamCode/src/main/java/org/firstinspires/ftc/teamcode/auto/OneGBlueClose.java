package org.firstinspires.ftc.teamcode.auto;

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
import org.firstinspires.ftc.teamcode.subsystems.sensors.Limelight;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="1 Gate Close - Blue", group = "BLUE")
@Config

public class OneGBlueClose extends LinearOpMode {
    public List<String> order1 = new ArrayList<>(Arrays.asList("P", "P", "G"));

    public List<String> targetOrder = order1; // default


    public static double[] start = new double[] { -65, -41.75, 0};

    //Obelisk look
    public static double[] lookAtOb = new double[] {-23,-23, -195};

    //Open Gate
    public static double[] openGatePos = new double[] {-7,-72+6+5.25-0.25, 135};



    //1st Spike!!
    public static double[] close1Shooting = new double[] {-38, -38, -135};
    public static double[] collect1Pre = new double[] { -12, -31, -90 };
    public static double[] collect1Mid = new double[] { -12, -22, -90 };

    public static double[] firstSpikeEnd = new double[] { -12, -53, -90 };
    public static double[] strafePos = new double[] { -17, -36, -90 };

    //2nd spike!!

    public static double[] collect2Mid = new double[] { 9, -25, -90 };
    public static double[] collect2Pre = new double[] { 9, -31, -90 };


    public static double[] secondSpikeEnd = new double[] { 9, -53, -90 };
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;
    private static class PARAMS{
        private double COLLECT_DRIVE_MAX_POWER = 0.18;
    }
    public static OneGBlueClose.PARAMS PARAMS = new OneGBlueClose.PARAMS();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20); // faster updates

        robot = new BrainSTEMRobot(hardwareMap, telemetry, this, createPose(start));
        AutoActions.setRobot(robot);



        DrivePath driveToOb = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(lookAtOb)).setMaxLinearPower(1)
        );

        DrivePath openGate = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(openGatePos)).setMaxLinearPower(0.5).setMaxTime(1.5)
        );


        DrivePath driveToPreloadShoot = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting))
        );

        DrivePath driveToShootOne = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting))
        );

        DrivePath driveToShootTwo = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2Mid)),
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
                new Waypoint(createPose(collect2Mid)),
                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(0.8),
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

//                        new SleepAction(0.5),

                        new SleepAction(0.7),



                        AutoActions.rampUp(),
                        new SleepAction(0.2),

                        AutoActions.moveSpindexer360(),
                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle(),




                        // FIRST SPIKE

                        new ParallelAction(
                                AutoActions.setCollectorOn(),
                                driveToCollectFirstSpikeEnd
                        ),


                        new ParallelAction(
                                openGate,
                                AutoActions.pivotClose(),
                                AutoActions.shooterTurnOnClose()
                        ),


                        AutoActions.waitForLimelightAuto(),

                        // ADD WAIT TIMES IF NEED HERE TODO


                        new ParallelAction(
                                driveToShootOne,
                                AutoActions.moveSpindexerMot(0, telemetry)
                        ),

                        AutoActions.setCollectorOff(),

                        new SleepAction(0.7),

                        AutoActions.rampUp(),
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
                                AutoActions.shooterTurnOnClose()
                                , AutoActions.pivotClose()
                                , driveToShootTwo,

                                AutoActions.moveSpindexerMot(2, telemetry)
                        ),


                        AutoActions.setCollectorOff(),

                        new SleepAction(0.7),



                        AutoActions.rampUp(),
                        new SleepAction(0.25),
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