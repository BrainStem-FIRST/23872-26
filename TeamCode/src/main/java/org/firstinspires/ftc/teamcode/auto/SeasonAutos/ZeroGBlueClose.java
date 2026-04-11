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
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="0 Gate Close - Blue", group = "BLUE")
@Config
public class ZeroGBlueClose extends LinearOpMode {

    public List<String> order1 = new ArrayList<>(Arrays.asList("P", "P", "G"));

    public List<String> targetOrder = order1; // default

    public static double[] start = new double[] { -65, -41.75, 0};

    //Obelisk look
    public static double[] lookAtOb = new double[] {-23,-23, -195};

    //Open Gate
    public static double[] openGatePos = new double[] {-7,-72+6+5.25, 135};



    //1st Spike!!
    public static double[] close1Shooting = new double[] {-39, -39, -135};
    public static double[] collect1Pre = new double[] { -12, -31, -90 };
    public static double[] collect1Mid = new double[] { -12, -22, -90 };

    public static double[] firstSpikeEnd = new double[] { -12, -58, -90 };
    public static double[] strafePos = new double[] { -17, -36, -90 };

    public static double[] collect3PrePass = new double[] { 12, -45, -90 };

    //2nd spike!!

    public static double[] collect2Mid = new double[] { 12, -25, -90 };
    public static double[] collect2Pre = new double[] { 12, -31, -90 };


    public static double[] secondSpikeEnd = new double[] { 12, -64, -90 };
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;

    public static class PARAMS{
        public double COLLECT_DRIVE_MAX_POWER = 0.15;
    }
    public static ZeroGBlueClose.PARAMS PARAMS = new ZeroGBlueClose.PARAMS();


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
                new Waypoint(createPose(collect3PrePass)),
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

        Action autoAction = new ParallelAction(
                new SequentialAction(
                        new ParallelAction(
                                AutoActions.shooterTurnOnClose()
                                , driveToOb
                        ),

                        AutoActions.waitForLimelightAuto(),


                        new ParallelAction(
                         driveToPreloadShoot,
                         AutoActions.moveSpindexerMot(0, telemetry)
                        ),



                        AutoActions.rampUp(),
//                            new SleepAction(0.2),
                          new SleepAction(0.2),
                       AutoActions.moveSpindexer360(),


                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle(),



                        //1st Spike Does Work ==========================
                        new ParallelAction(
                                AutoActions.setCollectorOn(),
                                driveToCollectFirstSpikeEnd
                        ),


                        new ParallelAction(
                                AutoActions.setCollectorOff(),
                                AutoActions.pivotClose(),
                                AutoActions.shooterTurnOnClose()
                        ),


                        driveToShootOne,

                        AutoActions.moveSpindexerMot(1, telemetry),

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
                               , driveToShootTwo
                       ),


                        AutoActions.moveSpindexerMot(2, telemetry),

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