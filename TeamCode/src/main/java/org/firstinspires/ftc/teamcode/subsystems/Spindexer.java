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
import org.firstinspires.ftc.teamcode.utils.srs.SRSHub;
import org.firstinspires.ftc.teamcode.utils.Component;


@Config
public class Spindexer implements Component {
    public static double maxPowerErrorThreshold = 10000000, maxPower = 0.99;

    public int SPINDEXER_TIME;

    private double previousVelocity;
    private int lastGoodPosition;
    public boolean isUnjamming = false;
    public ElapsedTime spindexerTimer;
    public ElapsedTime antijamTimer;

    public double power;

    private boolean wasMoving = false;
    public boolean justFinishedMoving = false;

    public DcMotorEx spindexerMotor;
    private HardwareMap map;
    private Telemetry telemetry;

    public int rawEncoder, wrappedEncoder, rawHubEncoderPosition;
    public int startShootingEncoder;
    private int absoluteEncoderStartingOffset, wrapAroundOffset;

    private SRSHub hub;
    private double error, prevError;
    private final ElapsedTime dtTimer = new ElapsedTime();

    public boolean indexerCued;
    private BrainSTEMRobot robot;

    ElapsedTime jamTime;

   public boolean jammed = false;

    public int shotsFired = 0;

    private double kP;
    public double targetEncoder;
    private boolean firstUpdate = true;

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.robot = robot;

        spindexerMotor = map.get(DcMotorEx.class, "spindexerMotor");
        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        jamTime = new ElapsedTime();
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

    public void updateIndexerPosition(double dt) {
        if (Math.abs(error) < Constants.spindexerConstants.INDEXER_SMALL_KP_THRESHOLD)
            kP = Constants.spindexerConstants.INDEXER_SMALL_KP;
        else
            kP = Constants.spindexerConstants.INDEXER_BIG_KP;

        if (isStatic() ) {
            spindexerMotor.setPower(0);
            return;
        }

        if (Math.abs(error) > maxPowerErrorThreshold) {
            power = maxPower * Math.signum(error);
        } else {
            if (isStatic() || (isUnjamming && antijamTimer.milliseconds()>200 && antijamTimer.milliseconds() <500)  ){
                power = 0;
            } else {
                double proportionalPower = error * kP;
                double frictionPower = Math.signum(power) * Constants.spindexerConstants.INDEXER_KF;
                // custom kD
                double changeInAbsError = (Math.abs(error) - Math.abs(prevError)) / dt;
                double derivativePower = 0;
                if (changeInAbsError < 0)
                    derivativePower = Constants.spindexerConstants.INDEXER_KD * Math.abs(changeInAbsError) * -Math.signum(kP);
                telemetry.addData("spindexer friction power", frictionPower);
                telemetry.addData("spindexer proportional power", proportionalPower);
                telemetry.addData("spindexer derivative power", derivativePower);

                power = frictionPower + proportionalPower + derivativePower;
                power = Range.clip(power, -Constants.spindexerConstants.MAX_POWER, Constants.spindexerConstants.MAX_POWER);
            }
        }
        spindexerMotor.setPower(power);
    }
;
    public void setTargetAdj(int adjust) {
        targetEncoder += adjust;
    }


    @Override
    public void reset() {

    }

    @Override
    public void update() {
        double dt = dtTimer.seconds();
        dtTimer.reset();
        if (firstUpdate) {
            dt = 0.05;
            firstUpdate = false;
        }
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
        prevError = error;
        error = targetEncoder - getCurrentPosition();

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
            updateIndexerPosition(dt);
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

