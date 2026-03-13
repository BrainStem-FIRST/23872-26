package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.srs.SRSHub;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;


@Config
public class Spindexer implements Component {
    public static double maxPowerErrorThreshold = 200, maxPower = 0.99;

    public int SPINDEXER_TIME;

    private double previousVelocity;
    private int lastGoodPosition;
    public boolean isUnjamming = false;
    public ElapsedTime spindexerTimer;
    public ElapsedTime antijamTimer;

    public double power;

    private boolean wasMoving = false;
    public boolean justFinishedMoving = false;

    public PIDController spindexerPid;
    public DcMotorEx spindexerMotor;
    private HardwareMap map;
    private Telemetry telemetry;

    public int rawEncoder, wrappedEncoder, rawHubEncoderPosition;
    public int startShootingEncoder;
    private int absoluteEncoderStartingOffset, wrapAroundOffset;

    private SRSHub hub;
    private double error;

    public boolean indexerCued;
    private BrainSTEMRobot robot;

    ElapsedTime jamTime;

   public boolean jammed = false;

    public int shotsFired = 0;

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.robot = robot;

        spindexerMotor = map.get(DcMotorEx.class, "spindexerMotor");
        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        jamTime = new ElapsedTime();

        spindexerPid = new PIDController(
                Constants.spindexerConstants.INDEXER_KP,
                Constants.spindexerConstants.INDEXER_KI,
                Constants.spindexerConstants.INDEXER_KD
        );
        spindexerTimer = new ElapsedTime();
        antijamTimer = new ElapsedTime();
        spindexerTimer.startTime();

        SRSHub.Config config = new SRSHub.Config();

        config.setEncoder(
                6,
                SRSHub.Encoder.PWM
        );

        RobotLog.clearGlobalWarningMsg();

        hub = hardwareMap.get(
                SRSHub.class,
                "SRSHub"
        );

        hub.init(config);

        this.absoluteEncoderStartingOffset = -60; // position that the absolute encoder reads when the indexer is centered
    }

    public void updateIndexerPosition() {
        double error = (spindexerPid.getTarget() - getCurrentPosition());
        power = spindexerPid.update(getCurrentPosition());

        if (isStatic() ) {

            // (isUnjamming)

            spindexerMotor.setPower(0);
            return;
        }

        if (Math.abs(error) > maxPowerErrorThreshold) {
            spindexerMotor.setPower(maxPower * Math.signum(power));
        }
        else {
            power += Math.signum(power) * Constants.spindexerConstants.INDEXER_KF;
            power = Range.clip(power, -Constants.spindexerConstants.MAX_POWER, Constants.spindexerConstants.MAX_POWER);
            spindexerMotor.setPower(power);

        }

        if (Math.abs(error) > maxPowerErrorThreshold) {
            spindexerMotor.setPower(maxPower * Math.signum(power));
        } else {
            if (isStatic() || (isUnjamming && antijamTimer.milliseconds()>200 && antijamTimer.milliseconds() <500)  ){
                //
                spindexerMotor.setPower(0);
            } else {
                power += Math.signum(power) * Constants.spindexerConstants.INDEXER_KF;

                power = Range.clip(power, -Constants.spindexerConstants.MAX_POWER, Constants.spindexerConstants.MAX_POWER);
                spindexerMotor.setPower(power);
            }
        }
    }
;
    public void setTargetAdj(int adjust) {
        spindexerPid.setTarget(spindexerPid.getTarget() + adjust);
    }


    public void fineAdjInDir() {
        double error = spindexerPid.getTarget()-wrappedEncoder;
        if (!robot.ramp.isRampUp()) setTargetAdj((int) (-Math.signum(error)* 20)); // TODO: Make sure right direction
    }
    @Override
    public void reset() {

    }

    @Override
    public void update() {
        hub.update();

        rawHubEncoderPosition = hub.readEncoder(6).position;
        double prevEncoder = rawEncoder;
        rawEncoder = 1024 - (rawHubEncoderPosition + absoluteEncoderStartingOffset);
//        telemetry.addData("raw hub indexer encoder", rawHubEncoderPosition);

        double dif = rawEncoder - prevEncoder;
        if(Math.abs(dif) > 500) {
            wrapAroundOffset -= (int)(1024* Math.signum(dif) );
        }
        wrappedEncoder = rawEncoder + wrapAroundOffset;

        error = getCurrentPosition() - spindexerPid.getTarget();


        double currentAmps = spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS);

//        if (currentAmps > 7000 && spindexerMotor.getPower() > maxPower- 0.05 && !isUnjamming) {
//            telemetry.addLine("JAM DETECTED - STARTING COOLDOWN");
//            isUnjamming = true;
//            antijamTimer.reset();
//        }

//        if (isUnjamming && antijamTimer.milliseconds() > 1000) {
//            isUnjamming = false;
//        }

        if (isJammed() && !jammed) {
            jammed = true;
            jamTime.reset();
        }
//
        if (jammed) {
            spindexerMotor.setPower(0);
            if (jamTime.milliseconds() > 500) jammed = false;
        } else {
            updateIndexerPosition();
        }





        boolean isCurrentlyMoving = !isStatic();
        justFinishedMoving = wasMoving && !isCurrentlyMoving;
        wasMoving = isCurrentlyMoving;
    }

    public boolean isJammed() {
        return spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS) > 7000 && Math.abs(spindexerMotor.getPower()) > Math.abs(maxPower- 0.05);

    }

    public int getCurrentPosition() {
        return wrappedEncoder;
    }

    public int getRawPosition() {
        return hub.readEncoder(6).position;
    }






    public double getError() {
        return Math.abs(error);
    }
    public boolean isStatic() {
        return getError() < Constants.spindexerConstants.ERROR_THRESHOLD;
    }
    @Override
    public String test() {
        return null;
    }


}

