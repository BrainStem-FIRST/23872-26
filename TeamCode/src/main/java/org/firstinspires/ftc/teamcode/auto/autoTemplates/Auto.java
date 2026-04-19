package org.firstinspires.ftc.teamcode.auto.autoTemplates;

import static org.firstinspires.ftc.teamcode.auto.autoTemplates.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.auto.autoTemplates.Alliance.RED;
import static org.firstinspires.ftc.teamcode.utils.pidDrive.UtilFunctions.createPose;
import static org.firstinspires.ftc.teamcode.utils.pidDrive.UtilFunctions.createRedPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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
import org.firstinspires.ftc.teamcode.auto.autoTemplates.Alliance;

@Config
@Autonomous(name="AHHHHHHHHHHHHHH")
public class Auto extends LinearOpMode {

    boolean isBlue;
    public static Alliance currentAlliance = Alliance.BLUE;

    public static String autoSequence = "B P 1G 2GO E";

    public static class PARAMS {
        public double COLLECT_DRIVE_MAX_POWER = 0.18;
    }
    public static PARAMS PARAMS = new PARAMS();

    public List<String> order1 = new ArrayList<>(Arrays.asList("P", "P", "G"));
    public List<String> targetOrder = order1; // Default

    private BrainSTEMRobot robot;

    // start + msc
    public static double[] START = new double[] { -65, -41.75, 0};
    public static double[] LOOK_OB = new double[] {-23, -23, -195};
    public static double[] OPEN_GATE = new double[] {-7, -72 + 6 + 5.25 - 0.25, 135};
    public static double[] OPEN_GATE_OBELISK = new double[] {-7, -72 + 6 + 5.25 - 0.25, 180};
    public static double[] END_POS = new double[] { -17, -36, -90 };

    // 1st Spike
    public static double[] CLOSE_SHOOT = new double[] {-38, -38, -135};
    public static double[] CLOSE_SHOOT_PASS = new double[] { -60, -60, -135 };
    public static double[] COLLECT_1_PRE = new double[] { -12, -31, -90 };
    public static double[] COLLECT_1 = new double[] { -12, -53, -90 };

    // 2nd Spike
    public static double[] COLLECT_2_MID = new double[] { 9, -25, -90 };
    public static double[] COLLECT_2_PRE = new double[] { 9, -31, -90 };
    public static double[] COLLECT_2 = new double[] { 9, -53, -90 };

    public static double[] COLLECT_3_PRE = new double[] { 36, -31, -90 };
    public static double[] COLLECT_3 = new double[] { 36, -53, -90 };



    private Pose2d start, openGateO, obelisk, openGate, end, shootClose, spike1Pre,  spike1, spike2Pre, spike2, shoot, shootPass, spike3Pre, spike3;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);



        robot = new BrainSTEMRobot(hardwareMap, telemetry, this, start);
        AutoActions.setRobot(robot);

        switch(currentAlliance) {
            case BLUE:
                isBlue = true;
                break;
            case RED:
                isBlue = false;
                break;
        }

        while(!isStarted() && !isStopRequested()) {
            if (autoSequence.toUpperCase().startsWith("B")) {
                currentAlliance = BLUE;
                isBlue = true;
            } else if (autoSequence.toUpperCase().startsWith("R")) {
                currentAlliance = RED;
                isBlue = false;
            }

            declarePoses();

            telemetry.addLine("AUTO IS READY");
            telemetry.addData("Alliance Detected", currentAlliance);
            telemetry.addData("Current Sequence", autoSequence);
            telemetry.update();
        }



        if (isStopRequested()) return;




        // drivepaths ==============================================================================
        DrivePath preloadShootDrive = new DrivePath(robot.drive, telemetry,
                new Waypoint(shoot)
        );
        DrivePath driveOffLine = new DrivePath(robot.drive, telemetry,
                new Waypoint(end)
        );


        List<Action> sequenceToRun = new ArrayList<>();

        boolean spike1Collected = false;
        boolean spike2Collected = false;
        boolean spike3Collected = false;


        // Split the string by spaces so we can read chunks like "1G" or "2GO"
        String[] instructions = autoSequence.toUpperCase().trim().split("\\s+");

        for (String i : instructions) {

            if (i.equals("P")) {
                // preload shoot
                sequenceToRun.add(new SequentialAction(
                        new ParallelAction(AutoActions.shooterTurnOnClose(), preloadShootDrive),
                        AutoActions.rampUp(),
                        new SleepAction(0.2),
                        AutoActions.moveSpindexer360(),
                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle()
                ));
            } else if (i.startsWith("1")) {
                // first spike
                boolean useGate = i.contains("G");
                boolean readOb = i.contains("O");

                String gateChoose = (useGate && readOb) ? "obelisk" : "none";

                sequenceToRun.add(firstSpikeCollectNShoot(useGate, readOb, gateChoose));
                spike1Collected = true;
            } else if (i.startsWith("2")) {
                // second spike
                boolean useGate = i.contains("G");
                boolean shouldReadOb = i.contains("O");

                String gateChoose = (useGate && !shouldReadOb) ? "obelisk" : "none";

                sequenceToRun.add(secondSpikeCollectNShoot(useGate, spike1Collected, shouldReadOb, gateChoose));
                spike2Collected = true;
            } else if (i.startsWith("3")) {
                // third spike
                boolean useGate = i.contains("G");
                boolean shouldReadOb = i.contains("O");

                String gateChoose = (useGate && !shouldReadOb) ? "obelisk" : "none";

                sequenceToRun.add(thirdSpikeCollectNShoot(useGate, spike1Collected, spike2Collected, shouldReadOb, gateChoose));
                spike3Collected = true;
            } else if (i.equals("E")) {
                // park
                sequenceToRun.add(driveOffLine);
            } else if (i.equals("G")) {
                // go directly to gat4e
            }

        }
        Action toRunAction = new ParallelAction(
                new SequentialAction(sequenceToRun), // Runs your built list
                AutoActions.robotUpdate(telemetry)   // Background Subsystem Updates
        );

        Actions.runBlocking(toRunAction);
    }


    private Action firstSpikeCollectNShoot(boolean gate, boolean shouldReadOb, String gateChoose) {
        Pose2d currentPose = robot.drive.localizer.getPose();
        Pose2d controlPoint = createControlPoint(currentPose, spike1);

        DrivePath collectionPath = new DrivePath(robot.drive, telemetry,
                new Waypoint(controlPoint)
                        .setMaxLinearPower(1.0)
                        .setPassPosition(true),
                new Waypoint(spike1Pre),
                new Waypoint(spike1)
                        .setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER)
                        .setMaxTime(4)
        );

        Action driveToGate;
        Action driveToShoot;
        boolean isObelisk = "obelisk".equalsIgnoreCase(gateChoose);

        if (gate) {
            Pose2d openGateChosen =

                    shouldReadOb ? openGate : openGateO;

            List<Waypoint> toGateWaypoints = new ArrayList<>();

            toGateWaypoints.add(new Waypoint(openGateChosen).setMaxTime(1.5));

            driveToGate = new DrivePath(robot.drive, telemetry, toGateWaypoints.toArray(new Waypoint[0]));
        } else {
            driveToGate = new SleepAction(0);
        }
        driveToShoot = new DrivePath(robot.drive, telemetry, new Waypoint(shoot));

        return new SequentialAction(
                new ParallelAction(
                        AutoActions.setCollectorOn(),
                        collectionPath
                ),
                new ParallelAction(
                        driveToGate,
                        AutoActions.pivotClose(),
                        AutoActions.shooterTurnOnClose()
                ),
                isObelisk ? AutoActions.waitForLimelightAuto() : new SleepAction(0),

                driveToShoot,
                AutoActions.setCollectorOff(),
                new SequentialAction(
                        AutoActions.rampUp(),
                        new SleepAction(0.2),
                        AutoActions.moveSpindexer360(),
                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle()
                )
        );
    }

    private Action secondSpikeCollectNShoot(boolean gate, boolean firstSpikeAlreadyCollected, boolean shouldReadOb, String gateChoose) {
        Pose2d currentPose = robot.drive.localizer.getPose();
        Pose2d controlPoint = createControlPoint(currentPose, spike2);

        DrivePath collectionPath = new DrivePath(robot.drive, telemetry,
                new Waypoint(controlPoint)
                        .setMaxLinearPower(1.0)
                        .setPassPosition(true),
                new Waypoint(spike2Pre),
                new Waypoint(spike2)
                        .setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER)
                        .setMaxTime(4)
        );

        Action driveToGate;
        Action driveToShoot;
        boolean isObelisk = "obelisk".equalsIgnoreCase(gateChoose);

        if (gate) {
            Pose2d openGateChosen = shouldReadOb ? openGate : openGateO;

            driveToGate = new DrivePath(robot.drive, telemetry,
                    new Waypoint(openGateChosen).setMaxTime(1.5)
            );

            if (firstSpikeAlreadyCollected) {
                driveToShoot = new DrivePath(robot.drive, telemetry, new Waypoint(shoot));
            } else {
                driveToShoot = new DrivePath(robot.drive, telemetry,
                        new Waypoint(shootPass).setPassPosition(true),
                        new Waypoint(shoot)
                );
            }
        } else {
            // No gate
            driveToGate = new SleepAction(0);

            if (firstSpikeAlreadyCollected) {
                driveToShoot = new DrivePath(robot.drive, telemetry, new Waypoint(shoot));
            } else {
                driveToShoot = new DrivePath(robot.drive, telemetry,
                        new Waypoint(shootPass).setPassPosition(true),
                        new Waypoint(shoot)
                );
            }
        }

        return new SequentialAction(
                new ParallelAction(
                        AutoActions.setCollectorOn(),
                        collectionPath
                ),
                new ParallelAction(
                        driveToGate,
                        AutoActions.pivotClose(),
                        AutoActions.shooterTurnOnClose()
                ),

                isObelisk ? AutoActions.waitForLimelightAuto() : new SleepAction(0),

                driveToShoot,
                AutoActions.setCollectorOff(),
                new SequentialAction(
                        AutoActions.rampUp(),
                        new SleepAction(0.2),
                        AutoActions.moveSpindexer360(),
                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle()
                )
        );
    }


    private Action thirdSpikeCollectNShoot(boolean gate, boolean spike1AlreadyCollected, boolean spike2AlreadyCollected, boolean shouldReadOb, String gateChoose) {
        Pose2d currentPose = robot.drive.localizer.getPose();

        Pose2d controlPoint = createControlPoint(currentPose, spike3);

        DrivePath collectionPath = new DrivePath(robot.drive, telemetry,
                new Waypoint(controlPoint)
                        .setMaxLinearPower(1.0)
                        .setPassPosition(true),
                new Waypoint(spike3Pre),
                new Waypoint(spike3)
                        .setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER)
                        .setMaxTime(4)
        );

        Action gatePath;
        Action shootPath;
        boolean isObelisk = "obelisk".equalsIgnoreCase(gateChoose);

        if (!spike1AlreadyCollected) {
            shootPath = new DrivePath(robot.drive, telemetry,
                    new Waypoint(shootPass).setPassPosition(true),
                    new Waypoint(shoot)
            );
        } else {
            shootPath = new DrivePath(robot.drive, telemetry,
                    new Waypoint(shoot)
            );
        }

        if (gate) {
            Pose2d openGateChosen = shouldReadOb ? openGate : openGateO;

            if (spike2AlreadyCollected) {
                gatePath = new DrivePath(robot.drive, telemetry,
                        new Waypoint(openGateChosen).setMaxTime(1.5)
                );
            } else {
                gatePath = new DrivePath(robot.drive, telemetry,
                        new Waypoint(spike2Pre).setSlowDownPercent(0.5).setPassPosition(true),
                        new Waypoint(openGateChosen).setMaxLinearPower(0.5).setMaxTime(1.5)
                );
            }
        } else {
            gatePath = new SleepAction(0);
        }

        return new SequentialAction(
                new ParallelAction(
                        AutoActions.setCollectorOn(),
                        collectionPath
                ),
                new ParallelAction(
                        gatePath,
                        AutoActions.pivotClose(),
                        AutoActions.shooterTurnOnClose()
                ),

                isObelisk ? AutoActions.waitForLimelightAuto() : new SleepAction(0),

                new ParallelAction(
                        AutoActions.setCollectorOff(),
                        shootPath
                ),

                new SequentialAction(
                        AutoActions.rampUp(),
                        new SleepAction(0.2),
                        AutoActions.moveSpindexer360(),
                        AutoActions.rampDown(),
                        AutoActions.turnShooterOnIdle()
                )
        );
    }


    public static Pose2d createControlPoint(Pose2d start, Pose2d end) {
        double dynamicX = (start.position.x + end.position.x) / 2.0;
        double dynamicY = (2*end.position.y + start.position.y) / 3;

        return new Pose2d(dynamicX, dynamicY, end.heading.toDouble());
    }

    public void declarePoses() {
        start = getPoseSide(START);
        obelisk = getPoseSide(LOOK_OB);
        openGateO = getPoseSide(OPEN_GATE_OBELISK);
        openGate = getPoseSide(OPEN_GATE);
        end = getPoseSide(END_POS);
        spike1Pre = getPoseSide(COLLECT_1_PRE);
        spike1 = getPoseSide(COLLECT_1);
        spike2Pre = getPoseSide(COLLECT_2_PRE);
        spike2 = getPoseSide(COLLECT_2);
        shoot = getPoseSide(CLOSE_SHOOT);
        shootPass = getPoseSide(CLOSE_SHOOT_PASS);
        spike3Pre = getPoseSide(COLLECT_3_PRE);
        spike3 = getPoseSide(COLLECT_3);
    }

    public Pose2d getPoseSide(double[] bluePose) {
        return isBlue ? createPose(bluePose) : createRedPose(bluePose);
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