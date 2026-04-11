package org.firstinspires.ftc.teamcode.auto.helping_opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@Config
@Disabled
@TeleOp (name = "Shooter Tuning")
public class ShooterTuning extends LinearOpMode {

    public static double targetVel = 1500;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.00015;


    public static boolean powerMotors = true;
    /*

    F - > P - > D

    high p val possible
    if overshoots a little add d

    1. SET kF (The Foundation)
 * - Set P, I, D to 0.
 * - Increase kF until Actual Velocity matches Target Velocity on the graph.
 * - STARTING VAL: 12.11 (Calculated for 5800 RPM @ 28 ticks/rev).
 * - DONE WHEN: The "lazy" speed of the motor settles right on the target.

 * * 2. SET kP (The Recovery Kick)
 * - Fire a shot and watch the "dip" in the graph.
 * - Increase kP by 5.0 increments (Start at 10.0, aim for 25.0 - 40.0).
 * - DONE WHEN: The velocity line "snaps" back to the target almost
 * instantly after a shot without "purring" (vibrating) while idle.

 * * 3. SET kD (The Stabilizer)
 * - If kP causes the velocity to "overshoot" (jump above the target)
 * after recovery, add kD.
 * - STARTING VAL: 1.0 - 3.0.
 * - DONE WHEN: The recovery curve is "critically damped"—it hits the
 * target line perfectly without a bump or oscillation.

 * * 4. SET kI (The Danger Zone)
 * - LEAVE AT 0.0. Integral windup causes erratic over-revving and
 * inconsistent shots during rapid fire.
 * * FINAL TEST: Spam the fire button. If the graph stays a flat line
 * with only tiny blips, you have officially beaten External PID.

     */


    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;

    public PIDController shooterPID;

    @Override
    public void runOpMode() {
        shooterMotorOne = hardwareMap.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "shooterMotorTwo");

        shooterMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
//            PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);

//            shooterMotorOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//            shooterMotorTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//
//            shooterMotorTwo.setVelocity(targetVel);
//            shooterMotorOne.setVelocity(targetVel);

            shooterPID = new PIDController(kP, kI, kD);
            shooterPID.setInputBounds(0, Constants.shooterConstants.MAX_TICKS_PER_SEC);
            shooterPID.setOutputBounds(0,1);


            setBothMotorVelocities(targetVel);

//            TelemetryPacket packet = new TelemetryPacket();
//            packet.put("target velocity", targetVel);
//            packet.put("actual velocity 1", shooterMotorOne.getVelocity());
//            packet.put("actual velocity 2", shooterMotorTwo.getVelocity());
            telemetry.addData("target velocity", targetVel);
            telemetry.addData("actual velocity 1", shooterMotorOne.getVelocity());
            telemetry.addData("actual velocity 2", shooterMotorTwo.getVelocity());
            telemetry.update();
        }



    }

    public void setBothMotorVelocities(double targetVelocity) {
        shooterPID.setTarget(targetVelocity);
        double error = targetVelocity - Math.abs(shooterMotorOne.getVelocity());

        double pidOutput = shooterPID.updateWithError(error);

        double shooterPower = pidOutput + kV * targetVelocity;

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
}
