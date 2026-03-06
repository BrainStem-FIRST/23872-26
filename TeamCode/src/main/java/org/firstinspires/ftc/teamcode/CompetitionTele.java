package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.PIDController;

/*
What to do:

RETUNE COLOR SENSOR

AUTO ADJUSTING BUTTON THAT ADJUSTS IN THE RIGHT DIRECTION, WHEN CLOSE ENOUGH GP RUMBLEs

INTAKE SLOWS WHEN A BALL IS TRYING TO BE TURNED

ADD ELApSED TIMER AFTER DETECTING BALL AND BEOFRE SHOOTING

ADD DIFF BUTTONS TO SHOOT ALL MOTIFS


AUTO PIVOTING CODE

FIX ANTIJAM



P1: auto align - according to dante spins in one direction for eternity && auto
    wrap issue? try update with error
 */
@TeleOp(name = "Competition Tele Yay")
public class CompetitionTele extends LinearOpMode {
    private GamepadTracker gp1;
    private GamepadTracker gp2;
    private BrainSTEMRobot robot;
    private PIDController alignmentPID;

    private boolean shooterOn;
    private boolean collectorOn = false;

    boolean wasHit = false;

    boolean thisJammed;
    ElapsedTime thisJamTime;


    ElapsedTime pressedTime;



    Vector2d goal = new Vector2d(-72, 72); //default: red
    private boolean red = true;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(BrainSTEMRobot.autoX, BrainSTEMRobot.autoY, BrainSTEMRobot.autoH));

//        robot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(0, 0, 0));

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);

        pressedTime = new ElapsedTime();
        thisJamTime = new ElapsedTime();


        robot.shooter.setShooterOff();

        alignmentPID = new PIDController(
                Constants.DriveConstants.ALIGNMENT_KP,
                Constants.DriveConstants.ALIGNMENT_KI,
                Constants.DriveConstants.ALIGNMENT_KD
        );






        telemetry.addLine("Robot is Ready!");

        telemetry.addData("Limelight connectivty", robot.limelight.limelight.isConnected());
        telemetry.update();

        waitForStart();

        while (!opModeIsActive()) {
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            robot.update();
            gp1.update();
            gp2.update();

            updateD1Drive();
            updateD1Buttons();

            updateDriver2();



        }
    }


    private void updateD1Drive() {

        // DRIVING ==========================================
        double y = -gamepad1.left_stick_y * 0.99;
        double x = gamepad1.left_stick_x * 0.99;
        double rx = gamepad1.right_stick_x * 0.75;

        if (gamepad2.right_bumper) {
            double dx = goal.x - robot.drive.localizer.getPose().position.x;
            double dy = goal.y - robot.drive.localizer.getPose().position.y;

            double targetAngle = Math.atan2(dy, dx);
            double currentHeading = robot.drive.localizer.getPose().heading.toDouble();
            double error = Angle.normDelta(targetAngle - currentHeading);

            alignmentPID.setTarget(Angle.normDelta(targetAngle));

//            rx = alignmentPID.updateWithError(error); // TODO: TEST THIS

            double pidOut = alignmentPID.update(currentHeading);

            double power = pidOut + Math.signum(pidOut) * 0.12;

            if (Math.abs(error) < Math.toRadians(1)) {
                power = 0;
            }

            power = Range.clip(power, -0.99, 0.99);
            rx = power;
        }


        robot.drive.setMotorPowers(
                y + x + rx,
                y - x - rx,
                y - x + rx,
                y + x - rx
        );
    }


    // D1 SUBSYSTEM CONTROLS =====================================================
    private void updateD1Buttons() {


        if (gamepad1.left_trigger > 0.1) {
            robot.collector.collectorState = Collector.CollectorState.EXTAKE;
        }
        else if (gamepad1.right_trigger > 0.1) {
            robot.collector.collectorState = Collector.CollectorState.INTAKE;
        }
        else {
            robot.collector.collectorState = Collector.CollectorState.OFF;
        }


        if (gp1.isFirstLeftBumper()) {
            robot.spindexer.setTargetAdj(Constants.spindexerConstants.TICKS_120);
        }

        if (gp1.isFirstRightBumper() && !wasHit) {
            robot.ramp.setRampUp();
            pressedTime.reset();
            // TODO: Test if works

            if (robot.shooter.isUpToSpeed()) {
                robot.hit = true;
            } else {
                robot.hit = false;
                gamepad1.rumble(500);
            }
        }


        if (robot.hit && pressedTime.milliseconds() > 250) {
            robot.spindexer.startShootingEncoder = robot.spindexer.wrappedEncoder;
            robot.spindexer.setTargetAdj(Constants.spindexerConstants.TICKS_360);
            robot.hit = false;
            wasHit = true;
        }

        if ((wasHit && pressedTime.milliseconds() > 2500 && robot.shooter.isShootFar()) || (wasHit && pressedTime.milliseconds() > 1250 && robot.shooter.isShootClose())) {
            robot.ramp.setRampDown();
            robot.shooter.setShooterOff();
            wasHit = false;
        }

        // Switch goal - make so only presses in first 10 sec
        if (gamepad1.x){
            telemetry.addLine("Color is Blue");
            goal = new Vector2d(-72, -68.5); //  change for left hand bias
            red = false;
        }

        if (gp1.isFirstDpadRight()) {
            robot.ramp.setRampUp();
        } else if (gp1.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }

        if (gp2.isFirstDpadRight()) {
            robot.ramp.setRampUp();
        } else if (gp1.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }

        if (gamepad1.a) {
            robot.shooter.setShooterShootClose();
            robot.pivot.setPivotShootClose();
        }


        if (gp1.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }

        if (gp2.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }

    }

    private void updateDriver2() {






        // makes any shooter button pressed after turned on, turn it off

        if (gp2.isFirstY()) {
            robot.shooter.setShooterShootFar();
            robot.pivot.setPivotShootFar();

        } else if (gp2.isFirstA()) {
            robot.shooter.setShooterShootClose();
            robot.pivot.setPivotShootClose();

        } else if (gp2.isFirstB()) {
            robot.shooter.setShooterIdle();

        } else if (gp2.isFirstX()) {
            robot.shooter.setShooterOff();

        }

        if (gp2.isFirstLeftBumper()) {
            robot.spindexer.fineAdjInDir();
        }

    }

}