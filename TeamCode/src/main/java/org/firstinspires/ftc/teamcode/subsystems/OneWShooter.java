package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;





@Config
public class OneWShooter implements Component {
    public static double testingSpeed = 1300;

    private final Telemetry telemetry;

    public static boolean powerMotors = true;
    public double closeTargetSpeed;

    // hardware constants

    private HardwareMap map;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;


    public ShooterState shooterState;

    //PID Controllers
    public PIDController shooterPID;

    public double targetVel;
    public double currentVel1;
    public double currentVel2;

    public double error1;
    public double error2;

    public int shotsFired = 0;
    private boolean wasAtSpeed = false;

    public enum ShooterState {
        OFF,
        IDLE,
        POINT,
        SHOOT_FAR,
        SHOOT_CLOSE,
        AUTO
    }


    private final BrainSTEMRobot robot;
    public OneWShooter(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.robot = robot;
        this.map = hardwareMap;
        this.telemetry = telemetry;

        shooterMotorOne = hardwareMap.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "shooterMotorTwo");



        shooterMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

//
        shooterPID = new PIDController(Constants.shooterConstants.kP_ONE, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
        shooterPID.setInputBounds(0, Constants.shooterConstants.MAX_TICKS_PER_SEC);
        shooterPID.setOutputBounds(0,1);

        shotsFired = 0;

//        PIDFCoefficients newPIDF = new PIDFCoefficients(
//                Constants.shooterConstants.kP_ONE,
//                Constants.shooterConstants.kI,
//                Constants.shooterConstants.kD,
//                Constants.shooterConstants.kF
//        );
//        shooterMotorOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
//        shooterMotorTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);


        this.shooterState = ShooterState.OFF;
    }



    @Override
    public void reset() {
       shooterPID.reset();
    }


    @Override
    public void update() {
        switch (shooterState) {
            case OFF:
                shooterMotorOne.setPower(-0.07); // -0.07
                shooterMotorTwo.setPower(-0.07);

                targetVel = 0;
                break;
            case SHOOT_FAR:
                shooterPID.setPIDValues(Constants.shooterConstants.kP_TWO, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
                setBothMotorVelocities(Constants.shooterConstants.FAR_SHOOT_VEL);
                targetVel = Constants.shooterConstants.FAR_SHOOT_VEL;

                break;

            case POINT:
                shooterPID.setPIDValues(Constants.shooterConstants.kP_TWO, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
                setBothMotorVelocities(Constants.shooterConstants.POINT_SHOOT_VEL);
                targetVel = Constants.shooterConstants.POINT_SHOOT_VEL;

                break;
            case SHOOT_CLOSE:
                shooterPID.setPIDValues(Constants.shooterConstants.kP_ONE, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
                targetVel = closeTargetSpeed;
                setBothMotorVelocities(targetVel);
//                setBothMotorVelocities(testingSpeed);

                break;

            case IDLE:
                shooterMotorOne.setPower(Constants.shooterConstants.IDLE_POWER);
                shooterMotorTwo.setPower(Constants.shooterConstants.IDLE_POWER);
                targetVel = 0;

                break;

            case AUTO:
                setBothMotorVelocities(Constants.shooterConstants.CLOSE_SHOOT_VEL);
                targetVel = Constants.shooterConstants.AUTO_VEL;
                break;

        }

        currentVel1 = Math.abs(shooterMotorOne.getVelocity());
        currentVel2 = Math.abs(shooterMotorTwo.getVelocity());

        error1 = Math.abs(currentVel1 - targetVel);
        error2 = Math.abs(currentVel2 - targetVel);

        // PIVOT
        double currentVel = shooterMotorOne.getVelocity();
        boolean isAtSpeed = Math.abs(currentVel - targetVel) < 50;


    }

    public boolean isUpToSpeed() {
        return (Math.abs(shooterMotorOne.getVelocity() - shooterPID.getTarget()) < 50) && shooterPID.getTarget()!= 0;
    }


    public void setBothMotorVelocities(double targetVelocity) {
        shooterPID.setTarget(targetVelocity);
        double error = targetVelocity - Math.abs(shooterMotorOne.getVelocity());

        double pidOutput = shooterPID.updateWithError(error);

        double shooterPower = pidOutput + Constants.shooterConstants.kV1 * targetVelocity;

        shooterPower = Range.clip(shooterPower, 0, Constants.shooterConstants.MAX_POWER);

        if (powerMotors) {
            shooterMotorOne.setPower(shooterPower);
            shooterMotorTwo.setPower(shooterPower);
        }

        telemetry.addData("Shooter Target Vel", targetVelocity);
        telemetry.addData("shooter error 1", error);
        telemetry.addData("shooter motor one velocity", shooterMotorOne.getVelocity());
        telemetry.addData("shooter motor two velocity", shooterMotorTwo.getVelocity());

        telemetry.addData("Shooter PID output 1", pidOutput);
        telemetry.addData("Shooter total output 1", shooterPower);

        telemetry.addData("shooter one Power", shooterMotorOne.getPower());
        telemetry.addData("shooter two power", shooterMotorTwo.getPower());



    }

    public void resetShotCounter() {
        shotsFired = 0;
    }
    public void setBoth(double power) {
        shooterMotorTwo.setVelocity(power);
        shooterMotorOne.setVelocity(power);
    }
    public void setShooterShootFar() {
        shooterState = ShooterState.SHOOT_FAR;
        shooterPID.reset();
    }
    public void setShooterShootAuto() {
        shooterState = ShooterState.AUTO;
        shooterPID.reset();
    }
    public void setShooterShootClose() {
        shooterState = ShooterState.SHOOT_CLOSE;
        shooterPID.reset();
    }
    public void setShooterShootPoint() {
        shooterState = ShooterState.POINT;
        shooterPID.reset();
    }

    public boolean   isShootFar() {
        if (shooterState == ShooterState.SHOOT_FAR) {
            return true;
        }
        return false;
    }

    public boolean   isShootClose() {
        if (shooterState == ShooterState.SHOOT_CLOSE) {
            return true;
        }
        return false;
    }
    public void setShooterOff() {
        shooterState = ShooterState.OFF;
        shooterPID.reset();
    }

    public void setShooterIdle(){
        shooterState = ShooterState.IDLE;
        shooterPID.reset();
    }




    @Override
    public String test() {
        return null;
    }
}


