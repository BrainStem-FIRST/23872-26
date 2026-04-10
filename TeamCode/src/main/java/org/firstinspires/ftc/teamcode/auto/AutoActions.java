package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.sensors.Limelight;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;


public class AutoActions {

    public ElapsedTime spindTime = new ElapsedTime();





    private static BrainSTEMRobot robot = null;
    public static void setRobot(BrainSTEMRobot robot) {
        AutoActions.robot = robot;
    }
    public static Action setCollectorOn() {
        return telemetryPacket -> {
            robot.collector.collectorState = Collector.CollectorState.AUTO;
            return false;
        };
    }

    public static Action setCollectorOff() {
        return telemetryPacket -> {
            robot.collector.collectorState = Collector.CollectorState.OFF;
            return false;
        };
    }

    public static Action robotUpdate(Telemetry telemetry) {
        return telemetryPacket -> {
            robot.update();
            Pose2d robotPose = robot.drive.localizer.getPose();
            BrainSTEMRobot.autoX = robotPose.position.x;
            BrainSTEMRobot.autoY = robotPose.position.y;
            BrainSTEMRobot.autoH = robotPose.heading.toDouble();
            return true;
        };
    }




    // SHOOTER

    public static Action shooterTurnOnFar() {
        return telemetryPacket -> {
            robot.shooter.setShooterShootFar();
            telemetryPacket.addLine("Shooter On");
            return false;
        };
    }

    public static Action shooterTurnOnClose() {
        return telemetryPacket -> {
            robot.shooter.setShooterShootAuto();
            telemetryPacket.addLine("Shooter On");
            return false;
        };
    }

    public static Action turnShooterOnIdle() {
        return telemetryPacket -> {
            robot.shooter.setShooterIdle();
            telemetryPacket.addLine("Shooter Idle");
            return false;
        };
    }

    public static Action shooterTurnOff() {
        return telemetryPacket -> {
            robot.shooter.setShooterOff();
            telemetryPacket.addLine("Shooter Off");
            return false;
        };
    }

    // SPINDEXER

    public static Action moveSpindexer120() {
        return moveSpindexer(Constants.spindexerConstants.TICKS_120);
    }

    public static Action moveSpindexer360() {
        return moveSpindexer360Hi();
    }

    public static Action resetOdoWLime(){
        return robot.limelight.resetOdoWLime();
    }

    public static Action limelightPoseReset() {
        return new InstantAction(() -> robot.limelight.resetOdoWLime());
    }



    private static Action moveSpindexer360Hi() {
        return new Action() {
            boolean first = true;
            double minTime = 0.7;
            double maxTime = 1;
            final ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    robot.spindexer.setTargetAdj(1024+341+341);
                    timer.reset();
                    first = false;
                }

                if (timer.seconds() <= minTime){
                    return true;}
                else if (timer.seconds() > maxTime)
                { return false;}

                TelemetryPacket packet = new TelemetryPacket();
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                return !robot.spindexer.isStatic();
            }


        };
    }


    public static Action moveSpindexer60() {
        return moveSpindexer(Constants.spindexerConstants.TICKS_60);
    }

    private static Action moveSpindexer(int ticks) {
        return new Action() {
            boolean first = true;
            double maxTime = 2;
            final ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    robot.spindexer.setTargetAdj(ticks);
                    timer.reset();
                    first = false;
                }

                if (timer.seconds() >= maxTime)
                    return false;

                TelemetryPacket packet = new TelemetryPacket();
                packet.addLine("move spindexer " + ticks);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                return !robot.spindexer.isStatic();
            }


        };
    }

    public static Action moveSpindexerMot(int num, Telemetry telemetry) {
        return new Action() {
            boolean first = true;
            double maxTime = 2;
            final ElapsedTime timer = new ElapsedTime();
            Action rotateIndexer;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                int ticks;
                if (first) {
                    ticks = robot.limelight.motifRotation(num);
//                    robot.spindexer.setTargetAdj(ticks);
                    timer.reset();
                    first = false;

                    TelemetryPacket packet = new TelemetryPacket();
//                    packet.addLine("move spindexer " + ticks);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    rotateIndexer = AutoActions.moveSpindexer(ticks);
//                    throw new RuntimeException("pattern: " + Arrays.toString(robot.limelight.targetOrder.toArray()) + ", ticks: " + ticks + ", spidnexer error: " + robot.spindexer.getError());
                }
                return rotateIndexer.run(telemetryPacket);
//
//                if (timer.seconds() >= maxTime)
//                    return false;
//
//                telemetry.addData("spindexer is static", robot.spindexer.isStatic());
//                telemetry.addData("spindexer error", robot.spindexer.getError());
//                telemetry.addData("spindexer power", robot.spindexer.spindexerMotor.getPower());
//                return !robot.spindexer.isStatic();
            }


        };
    }
    public static Action moveSpindexerMot(int num) {
        return new Action() {
            boolean first = true;
            double maxTime = 2;
            final ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    int ticks = robot.limelight.motifRotation(num);
                    robot.spindexer.setTargetAdj(ticks);
                    timer.reset();
                    first = false;

                    TelemetryPacket packet = new TelemetryPacket();
//                    packet.addLine("move spindexer " + ticks);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
//                    throw new RuntimeException("pattern: " + Arrays.toString(robot.limelight.targetOrder.toArray()) + ", ticks: " + ticks + ", spidnexer error: " + robot.spindexer.getError());
                }

                if (timer.seconds() >= maxTime)
                    return false;
                return !robot.spindexer.isStatic();
            }


        };
    }

    public static Action waitForLimelightAuto() {
        return new Action() {
            private ElapsedTime timer;
            private boolean isFirst = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(isFirst) {
                    timer = new ElapsedTime();
                    timer.reset();
                    isFirst = false;

                }

                robot.limelight.updateObeliskColors();
                robot.limelight.updateTargetMotif();

                boolean ready = Limelight.feducialResult >= 0;
                return timer.seconds() < 2 && !ready ;
            }
        };
    }

    // RAMP
    public static Action rampUp() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.ramp.setRampUp();
                    return false;
                }
        );
    }
    public static Action rampDown() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.ramp.setRampDown();
                    return false;
                }
        );
    }

    // PIVOT

    public static Action pivotClose() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.pivot.setPivotShootAuto();
                    return false;
                }
        );
    }

    public static Action pivotFar() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.pivot.setPivotShootFar();
                    return false;
                }
        );
    }


    // OTHER

    public static Action shootAll() {
        return new SequentialAction (
                telemetryPacket -> {
//                    toStartOfPatternShoot();
                    robot.ramp.setRampUp();
//                    new SleepAction(0.2);
                    new SleepAction(2.0);
                    robot.spindexer.setTargetAdj(1024);
//                    new SleepAction(0.5);
                    new SleepAction(2.0);
                    robot.shooter.setShooterIdle();
                    return false;
                }
        );
    }

    public static Action toStartOfPatternShoot() {
        return new Action() {
            boolean frist = true;
            final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (frist) {
                    int rotationAmount = robot.limelight.ballTrackerNew.getBestRotation();

                    if (rotationAmount == 0) {
                        return false;
                    }

                    robot.spindexer.setTargetAdj(rotationAmount);
                    timer.reset();
                    frist = false;

                }

                if (timer.seconds() >= 2.0) {
                    return false;
                }

                return !robot.spindexer.isStatic();
            }
        };
    }


    public Action triggerSpindexerAtPos(double targetY) {
        return new Action() {
            private boolean triggered = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!triggered && Math.abs(robot.drive.localizer.getPose().position.y - targetY) < 2.0) {
                    robot.spindexer.setTargetAdj(96);
                    triggered = true;
                }
                return !triggered;
            }
        };
    }



    public static Action waitForAccurateShooterVelocity() {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean first = true;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    timer.reset();
                    first = false;
                }

                if (timer.seconds() >= 2.0) {
                    return false;
                }


                // 4. Debugging Telemetry
                telemetryPacket.put("Shooter Target", robot.shooter.targetVel);
                telemetryPacket.put("Shooter V1", robot.shooter.currentVel1);
                telemetryPacket.put("Shooter V2", robot.shooter.currentVel2);
                telemetryPacket.put("Error V1", robot.shooter.error1);

                // 5. Check Threshold
                double threshold = 30;

                // Return true (keep running) if error is too high
                return (robot.shooter.error1 > threshold || robot.shooter.error2 > threshold);
            }
        };
    }
}
