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

public class CompetitionTele extends LinearOpMode {

    private Pose2d newPose;
    private GamepadTracker gp1;
    private Pose2d bluePose = new Pose2d((72-6.25), (72-7), Math.toRadians(-90));
    private Pose2d redPose = new Pose2d(72-6.25, -(72-7), Math.toRadians(90));

    private GamepadTracker gp2;
    private BrainSTEMRobot robot;
    private PIDController alignmentPID;

    private boolean shooterOn;
    private boolean collectorOn = false;

    boolean wasHit = false;
    boolean isHitting;

    boolean thisJammed;
    ElapsedTime thisJamTime;


    ElapsedTime pressedTime;
    ElapsedTime hittingTime;

    public boolean red;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(BrainSTEMRobot.autoX, BrainSTEMRobot.autoY, BrainSTEMRobot.autoH));
        robot.red = red;
//        robot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(0, 0, 0));

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);

        pressedTime = new ElapsedTime();
        thisJamTime = new ElapsedTime();
        hittingTime = new ElapsedTime();


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

            if (gamepad2.left_trigger > 0.13) {
                if (red){
                    newPose = redPose;
                } else {
                    newPose = bluePose;
                }
                robot.drive.localizer.setPose(newPose);
            }

        }
    }

    private void updateD1Drive() {

        double y = -gamepad1.left_stick_y * 0.99;
        double x = gamepad1.left_stick_x * 0.99;
        double rx = gamepad1.right_stick_x * 0.75;

        if (gamepad2.right_bumper) {
            Vector2d goalPosition = red ?
                    new Vector2d(Constants.shooterConstants.redGoalX, Constants.shooterConstants.redGoalY) :
                    new Vector2d(Constants.shooterConstants.blueGoalX, Constants.shooterConstants.blueGoalY);

            Vector2d robotToGoal = goalPosition.minus(robot.drive.localizer.getPose().position);

            double targetAngle = Math.atan2(robotToGoal.y, robotToGoal.x);
            double currentHeading = robot.drive.localizer.getPose().heading.toDouble();
            double error = Angle.normDelta(targetAngle - currentHeading);

            alignmentPID.setTarget(Angle.normDelta(targetAngle));

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
        } else if (gamepad1.right_trigger > 0.1) {
            robot.collector.collectorState = Collector.CollectorState.INTAKE;
        } else {
            robot.collector.collectorState = Collector.CollectorState.OFF;
        }

        if (gp1.isFirstLeftBumper()) {
            robot.spindexer.setTargetAdj(Constants.spindexerConstants.TICKS_120);
        }

        if (gp1.isFirstRightBumper() && !wasHit) {
            robot.ramp.setRampUp();
            pressedTime.reset();

            if (robot.shooter.isUpToSpeed()) {
                robot.hit = true;
            } else {
                robot.hit = false;
                gamepad1.rumble(500);
            }
        }

        if (robot.hit && pressedTime.milliseconds() > 250 && robot.shooter.isUpToSpeed()) {
            robot.spindexer.startShootingEncoder = robot.spindexer.wrappedEncoder;
            robot.spindexer.setTargetAdj(Constants.spindexerConstants.TICKS_360);
            robot.hit = false;
            wasHit = true;
        }

        if ((wasHit && pressedTime.milliseconds() > 2500 && (robot.shooter.isShootFar())) || (wasHit && pressedTime.milliseconds() > 2000 && robot.shooter.isShootClose())) {
            robot.ramp.setRampDown();
            wasHit = false;
            isHitting = true;
        } else {
            isHitting = false;
            hittingTime.reset();
        }

        if (isHitting && hittingTime.milliseconds() > 500) {
            robot.shooter.setShooterOff();
            isHitting = false;
        }

        if (gp1.isFirstX()) {
            robot.shooter.setShooterOff();
        }

        if (gp1.isFirstDpadRight()) {
            robot.ramp.setRampUp();
        } else if (gp1.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }

        if (gp1.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }

        if (gamepad1.a) {
            robot.shooter.setShooterShootClose();
            robot.pivot.setPivotShootClose();
        }

        if (gp1.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }
    }

    private void updateDriver2() {

        if (gp2.isFirstA()) {
            if (gamepad2.right_trigger > 0.2) {
                robot.shooter.setShooterShootAuto();
                robot.pivot.setPivotShootAuto();
            } else {
                robot.shooter.setShooterShootClose();
                robot.pivot.setPivotShootClose();
            }
        }

        if (gp2.isFirstY()) {

                robot.shooter.setShooterShootPoint();
                robot.pivot.setPivotShootPoint();
        }

        if (gp2.isFirstX()) {
            robot.shooter.setShooterOff();
        }

        if (gp2.isFirstDpadRight()) {
            robot.ramp.setRampUp();
        }

        if (gp2.isFirstDpadDown()) {
            robot.park.setParkDown();

        }

        if (gp2.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }

        if (gp2.isFirstLeftBumper()) {
            robot.park.setParkUp();
        }
    }
}