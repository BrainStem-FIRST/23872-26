package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.srs.SRSHub;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.OneWShooter;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
//import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.sensors.Limelight;
import org.firstinspires.ftc.teamcode.utils.BallSensor;
import org.firstinspires.ftc.teamcode.utils.BallTrackerNew;
import org.firstinspires.ftc.teamcode.utils.Component;

public class BrainSTEMRobot {

    //NEEDS TO CHOOSE ONE
    // TODO: Clean up for new subsystems

    public static double autoX, autoY, autoH;
    // Don't touch these
    public Telemetry telemetry;
    public OpMode opMode;
    private final ArrayList<Component> subsystems;
    public Spindexer spindexer;
    public Collector collector;
    public MecanumDrive drive;

    public Limelight limelight;
    public BallSensor ballSensor;
    public Pivot pivot;
    public Ramp ramp;
    public OneWShooter shooter;
    public BallTrackerNew ballTracker;
    private boolean goodToMove = false;
    private BallTrackerNew.BallColor detectedColor;
    private ElapsedTime ballDetectTimer = new ElapsedTime();




    public boolean hit = false;

    public boolean isSpindStopped;

    public boolean isNextEmpty;

    public int ballsShot = 0;

    private ElapsedTime lastShotTime;

    public boolean isAuto = false;




    private boolean checkingColorAfterMovingSpind = false;

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, Pose2d startPose) {

        this.telemetry = telemetry;
        this.opMode = opMode;


        ballSensor = new BallSensor(hwMap);
        subsystems = new ArrayList<>();

        spindexer = new Spindexer(hwMap, telemetry, this);
        collector = new Collector(hwMap, telemetry);
        shooter = new OneWShooter(hwMap, telemetry);
        ramp = new Ramp(hwMap, telemetry);
        pivot = new Pivot(hwMap, telemetry, shooter);
        limelight = new Limelight(hwMap, telemetry, this);

        ballTracker = new BallTrackerNew(spindexer);


        subsystems.add(limelight);

        subsystems.add(spindexer);
        subsystems.add(collector);
        subsystems.add(shooter);
//        subsystems.add(finger);
//        subsystems.add(shooterOne);
        subsystems.add(ramp);
        subsystems.add(pivot);

        // Defining the Motors
        drive = new MecanumDrive(hwMap,startPose);

        lastShotTime = new ElapsedTime();

    }




    public void update() {



        pivot.updateCompensatedPosition(ballsShot);




        for (Component c : subsystems) {
            c.update();
        }




        drive.localizer.update();




//        if (limelight != null) {
//            limelight.update();
//            limelight.updateObeliskColors();
//            telemetry.addData("Limelight fedu res", Limelight.feducialResult);
//        }
        isSpindStopped = (Math.abs(spindexer.spindexerPid.getTarget() - spindexer.getCurrentPosition())) < 50 || spindexer.spindexerMotor.getVelocity()<15;
        ballSensor.setIfIndexerIsMoving(!isSpindStopped);





        // DETECT BALL IF SPIND IS NOT MOVING
        if (isSpindStopped ) {

            String newBall = "EMPTY";
            newBall = ballSensor.scanForNewBall();

//            telemetry.addData("NEW BALL", newBall);


            if (newBall != null ) {

                BallTrackerNew.BallColor color = BallTrackerNew.BallColor.valueOf(newBall);

                BallTrackerNew.Slot collectSlot = limelight.ballTrackerNew.getSlotAtCollectPos();
                collectSlot.color = color;


//                telemetry.addLine("NOT NULL BALL");

                if (limelight.ballTrackerNew.isNextSlotEmpty()) {
                    spindexer.setTargetAdj(341);
                }
            }
//            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) ballSensor.colorSensor).getDistance(DistanceUnit.CM));
        }

        isNextEmpty = limelight.ballTrackerNew.isNextSlotEmpty();

        if (shooter.isShootFar() && ramp.isRampUp()) {
            ballsShot = getBallsShot();
        }
        else {
            ballsShot = 0;
        }





        if (shooter.isShootFar()) {
            Spindexer.maxPower = 0.5;}
        else if (shooter.isShootClose()){
            Spindexer.maxPower = 0.99;
        } else{
            Spindexer.maxPower = 0.99;
        }

        // TELEMETRY ===============================================================================
        allTelemetry();
        telemetry.update();
    }



    private void allTelemetry() {


        telemetry.addData("is jammed", spindexer.jammed);

        telemetry.addData("button status", hit);
        telemetry.addData("is shooter up to speed", shooter.isUpToSpeed());

        telemetry.addData("Limelight reading", String.valueOf(limelight.targetOrder));

        telemetry.addData("Balls Shot", ballsShot);

//        telemetry.addLine("\n=== BALL SENSOR DEBUG ===");
//        telemetry.addData("Beam State", !ballSensor.beamBreak.getState());
//        telemetry.addData("Timer (ms)", ballSensor.timer);
//        telemetry.addData("Delay Required ", BallSensor.delayTimeMs);
//        telemetry.addData("R/G/B %", String.format("%.2f / %.2f / %.2f", ballSensor.rPercent, ballSensor.gPercent, ballSensor.bPercent));
//
//        telemetry.addLine("\n=== DRIVE ===");
//        telemetry.addData("FL", drive.FL.getPower());
//        telemetry.addData("BL", drive.BL.getPower());
//        telemetry.addData("FR", drive.FR.getPower());
//        telemetry.addData("BR", drive.BR.getPower());

        telemetry.addData("TargetMotf", limelight.targetOrder);


        telemetry.addLine("=== SPINDEXER SLOTS ===");
        telemetry.addData("Slot A", String.format("%s @ %d ticks", limelight.ballTrackerNew.slotA.color, limelight.ballTrackerNew.slotA.currentAbsPos));
        telemetry.addData("Slot B", String.format("%s @ %d ticks", limelight.ballTrackerNew.slotB.color, limelight.ballTrackerNew.slotB.currentAbsPos));
        telemetry.addData("Slot C", String.format("%s @ %d ticks", limelight.ballTrackerNew.slotC.color, limelight.ballTrackerNew.slotC.currentAbsPos));

        telemetry.addLine("\n=== WHERE IS EACH SLOT ===");
        telemetry.addData("At Collect Pos", limelight.ballTrackerNew.getSlotAtCollectPos().name);
        telemetry.addData("At Shooting Pos", limelight.ballTrackerNew.getSlotAtShootingPos().name);

        telemetry.addLine("\n=== DETECTION STATES");
        telemetry.addData("Spind Stopped?", isSpindStopped);

//        telemetry.addData("Is Indexing?", ballSensor.isIndexing);
//        telemetry.addData("Good To Move?", goodToMove);
//        telemetry.addData("Detected Color", detectedColor);
//        telemetry.addData("Collect color - ball tracking", limelight.ballTrackerNew.thisBall);
//        telemetry.addData("Is next empty", isNextEmpty);
//        telemetry.addData("Color delay time", BallSensor.delayTimeMs);
//        telemetry.addData("from delay settle time", BallSensor.settleDelayMs);
//
//        telemetry.addLine("\n === PATTERN MATCHING ===");
//        telemetry.addData("Target Motif", limelight.ballTrackerNew.targetMotif);
//        telemetry.addData("Fiducial ID", Limelight.feducialResult);
//
//
//        telemetry.addLine("\n=== COLOR SENSOR ===");
//        telemetry.addData("R", ballSensor.rPercent);
//        telemetry.addData("G", ballSensor.gPercent);
//        telemetry.addData("B", ballSensor.bPercent);
//        telemetry.addData("Alpha", ballSensor.alpha);

        telemetry.addLine("\n=== LOCATION ===");
        telemetry.addData("Pose", drive.localizer.getPose().toString());
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("heading", drive.localizer.getPose().heading.toDouble());

        telemetry.addLine("\n=== SHOOTER ===");
        telemetry.addData("State", shooter.shooterState);
        telemetry.addData("Shooter Vel", shooter.shooterMotorOne.getVelocity());
        telemetry.addData("Shooter Target", shooter.shooterPID.getTarget());
        telemetry.addData("At Speed", Math.abs(shooter.shooterMotorOne.getVelocity() - shooter.targetVel) < 50);
        telemetry.addData("Giving power", shooter.shooterMotorOne.getPower());

        telemetry.addLine("\n=== SPINDEXER ===");
        telemetry.addData("Raw Encoder", spindexer.getRawPosition());
        telemetry.addData("Position", spindexer.getCurrentPosition());
        telemetry.addData("Target", spindexer.spindexerPid.getTarget());
        telemetry.addData("Power giving:", spindexer.spindexerMotor.getPower());
        telemetry.addData("Spindexer Current", spindexer.spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS));

        telemetry.addLine("\n=== HOOD ===");
        telemetry.addData("Pivot left pos", pivot.getLeftPos());
        telemetry.addData("Pivot right pos", pivot.getRightPos());
        telemetry.addData("Pivot state", pivot.pivotState);
        telemetry.addData("Pivot adjusted Pos", pivot.newPos);

        telemetry.addLine("\n=== INTAKE + RAMP ===");
        telemetry.addData("Collector", collector.collectorMotor.getVelocity());
        telemetry.addData("Ramp", ramp.rampState);

    }
    private int getBallsShot() {
        double diff = (spindexer.wrappedEncoder - spindexer.startShootingEncoder) / 1024. * 360;

        if ( diff >= 280) {
            return 3;
        } else if ( diff >= 160) {
            return 2;
        } else if (diff >= 40) {
            return 1;
        }
        return 0;
//        return ramp.isRampUp() && shooter.isShootFar() && (Math.abs(position - 200) < 15) && spindexer.spindexerMotor.getVelocity() > 10;
    }

}