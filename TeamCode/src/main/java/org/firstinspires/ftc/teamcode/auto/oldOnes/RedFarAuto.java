//package org.firstinspires.ftc.teamcode.auto;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.BrainSTEMAutoRobot;
//import org.firstinspires.ftc.teamcode.auto.AutoActions;
//import org.firstinspires.ftc.teamcode.auto_subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.auto_subsystems.Spindexer;
//
//@Autonomous (name = "Red Far Auto")
//@Config
//public final class RedFarAuto extends LinearOpMode {
//    public static class Positions {
//        public double startX = 62.6, startY = 16.6, startA = Math.toRadians(180);
//        public double preloadX = 49, preloadY = 11, preloadA = Math.toRadians(120), preloadT = Math.toRadians(120);
//        public double collect1X = 36, collect1Y = 28, collect1A = Math.toRadians(90), collect1T = Math.toRadians(90);
//        public double collect2X = 36, collect2Y = 30, collect2A = Math.toRadians(90), collect2T = Math.toRadians(90);
//        public double collect3X = 36, collect3Y = 32, collect3A = Math.toRadians(90), collect3T = Math.toRadians(90);
//
//    }
//    public static Positions positions = new Positions();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // DECLARE POSES
//        Pose2d beginPose = new Pose2d(positions.startX, positions.startY, positions.startA);
//        Pose2d shootPose = new Pose2d(positions.preloadX, positions.preloadY, positions.preloadA);
//        Pose2d collectPose = new Pose2d(positions.collect1X, positions.collect1Y, positions.collect1A);
//        Pose2d collectPose2 = new Pose2d(positions.collect2X, positions.collect2Y, positions.collect2A);
//        Pose2d collectPose3 = new Pose2d(positions.collect3X, positions.collect3Y, positions.collect3A);
//
//        BrainSTEMAutoRobot robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, beginPose);
//
//        Action preloadDrive = robot.drive.actionBuilder(beginPose)
//                .splineToLinearHeading(shootPose, positions.preloadT)
//                .build();
//
//        Action collectDrive = robot.drive.actionBuilder(shootPose)
//                .splineToLinearHeading(collectPose, positions.collect1T)
//                .build();
//
//        Action collectDrive2 = robot.drive.actionBuilder(collectPose)
//                .splineToLinearHeading(collectPose2, positions.collect2T)
//                .build();
//
//        Action collectDrive3 = robot.drive.actionBuilder(collectPose)
//                .splineToLinearHeading(collectPose3, positions.collect3T)
//                .build();
//
//        Action robotUpdate = new AutoActions().robotUpdate(robot);
//
//        Action moveSpindexer120 = new AutoActions().moveSpindexer120(robot);
//
//        Action moveSpindexer60 = new AutoActions().moveSpindexer60(robot);
//
//        Action shooterTurnOnFar = new AutoActions().shooterTurnOnFar(robot);
//
//        Action shooterTurnOff = new AutoActions().shooterTurnOff(robot);
//
//        Action fingerServoU = new AutoActions().fingerServoU(robot);
//
//        Action fingerServoD = new AutoActions().fingerServoD(robot);
//
//        Action setCollectorOn = new AutoActions().setCollectorOn(robot);
//
//        Action setCollectorOff = new AutoActions().setCollectorOff(robot);
//
//
//
//
//        waitForStart();
//
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        new Action[]{new SequentialAction(
//                                preloadDrive,
//                                shooterTurnOnFar,
//                                new SleepAction(2),
//                                fingerServoU,
//                                new SleepAction(2),
//                                fingerServoD,
//                                new SleepAction(2),
//                                moveSpindexer120,
//                                new SleepAction(2),
//                                fingerServoU,
//                                new SleepAction(2),
//                                fingerServoD,
//                                new SleepAction(2),
//                                moveSpindexer120,
//                                new SleepAction(2),
//                                fingerServoU,
//                                new SleepAction(2),
//                                fingerServoD,
//                                shooterTurnOff,
//                                setCollectorOn,
//                                new SleepAction(2),
//                                collectDrive,
//                                new SleepAction(2),
//                                collectDrive2,
//                                new SleepAction(2),
//                                collectDrive3,
//                                setCollectorOff
//
//
//
//
//                        ),
//                                robotUpdate})
//
//        );
//
//        while (opModeIsActive());
//
//
////            robot.shooter.setShooterShootFar();
////            robot.update();
////
////
////            wait(750);
////
////            robot.finger.fingerState = Finger.FingerState.UP;
////            robot.update();
////
////            wait(1000);
////
////            robot.finger.fingerState = Finger.FingerState.DOWN;
////            robot.update();
////
////            robot.shooter.setShooterOff();
////            robot.update();
////
////            robot.spindexer.rotateDegrees(60);
////            robot.update();
////
////
////        wait (100);
////
//        robot.update();
//
//
//        while (opModeIsActive());
//
//
//
//
//
//    }
//
////    public void wait(double time, BrainSTEMTeAutoRobot robot){
////        Actions.runBlocking(
////                robot.drive.actionBuilder(robot.drive.localizer.getPose())
////                        .waitSeconds(time)
////                        .build());
////    }
//
//}

