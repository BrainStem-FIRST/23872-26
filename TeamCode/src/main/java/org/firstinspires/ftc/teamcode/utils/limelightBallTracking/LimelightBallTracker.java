package org.firstinspires.ftc.teamcode.utils.limelightBallTracking;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.Component;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


@Config
public class LimelightBallTracker implements Component {
    // TODO: somehow account for when ball is moving
    public static double LATERAL_KP = 0.045; // need tuneing
    public static double LATERAL_KD = 0.008; // need tuning
    public static double FORWARD_KP = 0.028; // need tuning
    public static double LATERAL_DEADBAND_DEG = 3.0;
    public static double FORWARD_DEADBAND_IN = 2;
    public static double MAX_LATERAL_POWER = 0.45;
    public static double MAX_FORWARD_POWER = 0.38;
    public static double COLLECTOR_TRIGGER_IN = 4;
    public static double FORWARD_KD = 0.005;
    private double lastDirectError = 0.0;
    private final ElapsedTime timerForForwardD = new ElapsedTime();

    // camera and ball constants
    public static double CAMERA_HEIGHT_IN = 8;
    public static double CAMERA_PITCH_DEG = -17.0; // TODO: check if its positive or negative
    public static double CAMERA_FORWARD_OFFSET_IN = 7.125;
    public static double COLLECTOR_OFFSET_FROM_CAMERA_IN = 2.67;
    public static double BALL_DIAMETER_IN = 4.90;
    public static double FOCAL_LENGTH = 300.0; // CHANGE
    public static double H_FOV_DEG = 63.3;
    public static double V_FOV_DEG = 49.7;

    public final Limelight3A limelight;
    public int snapscriptPipeline = 1;
    public int apriltagPipeline = 0;


    public List<String> targetOrder;

    private final BrainSTEMRobot robot;
    private final Telemetry telemetry;


    public static int feducialResult = -1;

    private double lastTx = 0.0;
    private ElapsedTime timerForD = new ElapsedTime();


    public enum LLState {
        BALL_PIPELINE,
        SWITCHING_TO_TAG_PIPELINE,
        TAG_PIPELINE,
        SWITCHING_TO_BALL_PIPELINE
    }

    public LLState currentLLState = LLState.BALL_PIPELINE;
    private final double PIPELINE_SWITCH_DELAY_MS = 150.0;
    private ElapsedTime pipelineTimer;

    public LimelightBallTracker(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.robot = robot;

        limelight.pipelineSwitch(snapscriptPipeline);
        limelight.start();

        pipelineTimer = new ElapsedTime();
    }

    @Override
    public void update() {
        switch (currentLLState) {
            case SWITCHING_TO_TAG_PIPELINE:
                if (pipelineTimer.milliseconds() > PIPELINE_SWITCH_DELAY_MS)
                    currentLLState = LLState.TAG_PIPELINE;
                else
                    return;
                break;
            case SWITCHING_TO_BALL_PIPELINE:
                if (pipelineTimer.milliseconds() > PIPELINE_SWITCH_DELAY_MS)
                    currentLLState = LLState.BALL_PIPELINE;
                else
                    return;
                break;
            case BALL_PIPELINE:
                processBallTracking();
                break;
            case TAG_PIPELINE:
                processAprilTags();
                break;
        }



    }

    @Override
    public void reset() {
        lastTx = 0.0;
        timerForD.reset();
    }



    @Override
    public String test() { return null; }

    public Action alignLateral() {
        resetPID();
        return new Action() {
            ElapsedTime timer = null;
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                BallResult ball = getClosestBall();

                packet.put("Ball Detected", ball != null);

                if (ball == null) {
                    if (timer == null) {
                        timer = new ElapsedTime();
                    }

                    if (timer.seconds() > 1.0) {
                        packet.addLine("Ball lost ");
                        stopDriving();
                        return false;
                    }

                    stopDriving();
                    return true;
                }

                timer = null;

                double strafeError = calculateStrafingDistance(ball.ty, ball.tx);
                packet.addLine("strafeError:" + strafeError);

                if (Math.abs(strafeError) < LATERAL_DEADBAND_DEG) {
                    stopDriving();
                    lastTx = 0;
                    return false;
                }

                double power = calculateLateralPower(strafeError);

                setDrive(0, power);

                return true;
            }
        };
    }

    public Action driveIntoBall(Runnable collectorOn) {
        return new Action() {
            boolean collectorHasBeenTriggered = false;
            ElapsedTime lostBallTimer = null;
            double lastForward = MAX_FORWARD_POWER * 0.5;
            double lastLateral = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                BallResult ball = getClosestBall();

                packet.put("Collector State:", robot.collector.collectorState);

                if (ball == null) {
                    if (lostBallTimer == null) {
                        lostBallTimer = new ElapsedTime();
                    }

                    if (lostBallTimer.seconds() < 1) {
                        setDrive(lastForward, 0);
                        return true;
                    }

                    stopDriving();
                    return false;
                }

                lostBallTimer = null;

                packet.put("raw_ty", ball.ty);
                packet.put("raw_radius", ball.radius);
                packet.put("angle_deg", ball.ty + CAMERA_PITCH_DEG);


                packet.put("collectorTriggered", collectorHasBeenTriggered);
                packet.put("ballVisible", ball != null);

                double directDist = calculateDirectDistance(ball.ty, ball.tx);
                packet.put("direct_dist", directDist);

                if (directDist <= FORWARD_DEADBAND_IN) {
                    stopDriving();
                    robot.collector.off();
                    return false;
                }

                if (!collectorHasBeenTriggered && directDist < COLLECTOR_TRIGGER_IN && collectorOn != null) {
                    collectorOn.run();
                    collectorHasBeenTriggered = true;
                }

                double forward = calculateForwardPower(directDist);

                double strafeError = calculateStrafingDistance(ball.ty, ball.tx);
                double lateral = calculateLateralPower(strafeError) * 0.4;

                setDrive(forward, lateral);

                return true;
            }
        };
    }
    public Action driveIntoBall() {
        return driveIntoBall(null);
    }


    public Action fullCollect(Runnable collectorOn) {
        resetPID();
        return new SequentialAction(
                alignLateral(),
                driveIntoBall(collectorOn)
        );
    }

    // closest to ll output stuff
    private void processBallTracking() {
        BallResult ball = getClosestBall();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas overlay = packet.fieldOverlay();

        if (ball == null) {
            telemetry.addLine("[llight] no ball detected");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            return;
        }

        Pose2d robotPose = robot.drive.localizer.getPose();
        double[] fieldPos = ballToFieldCoords(robotPose, ball.ty, ball.tx);

        overlay.setFill("green");
        overlay.fillCircle(fieldPos[0], fieldPos[1], 2.45);

        packet.put("[liemlight] balls", ball.numBalls);
        packet.put("[limelight] tx (deg)", String.format("%.2f", ball.tx));
        packet.put("[limelight] ty (deg)", String.format("%.2f", ball.ty));
        packet.put("[limelight] dist (in)", String.format("%.2f", ball.distance));
        packet.put("[limelight] field x", String.format("%.2f", fieldPos[0]));
        packet.put("[limelihgt] field y", String.format("%.2f", fieldPos[1]));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public BallResult getClosestBall() {
        LLResult result = limelight.getLatestResult();
        if (result == null)
            return null;

        double[] output = result.getPythonOutput();
        if (output == null || output.length < 7)
            return null;

        double tv = output[0];
        int numBalls = (int) output[1];
        if (tv < 0.5 || numBalls <= 0)
            return null;


        double radius = output[4];
        double tx = output[5];
        double ty = output[6];
        double dist = calculateBallDistance(ty);
        // double dist = Math.abs(ty) > 1.0 ? distanceFromTy(ty) : distanceFromPixelWidth(radius * 2.0);

        return new BallResult(tx, ty, dist, radius, numBalls);
    }

    public BallResult[] getAllBalls() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return new BallResult[0];

        double[] output = result.getPythonOutput();
        if (output == null || output.length < 3) return new BallResult[0];

        int numBalls = (int) output[1];
        if (numBalls <= 0) return new BallResult[0];

        BallResult[] balls = new BallResult[numBalls];
        for (int n = 0; n < numBalls; n++) {

            int base = 2 + (n * 5);
            if (output.length < base + 5) break;

            double radius = output[base + 2];
            double tx = output[base + 3];
            double ty = output[base + 4];
            double dist = Math.abs(ty) > 1.0 ? calculateBallDistance(ty) : distanceFromPixelWidth(radius * 2.0);

            balls[n] = new BallResult(tx, ty, dist, radius, numBalls);
        }
        return balls;
    }


    // distance helper

    public double calculateBallDistance(double ty) {
        double totalAngleDeg = -(ty + CAMERA_PITCH_DEG);

        if (totalAngleDeg <= 0.1) return 999.0;

        double angleRad = Math.toRadians(totalAngleDeg);
        double height = CAMERA_HEIGHT_IN - (BALL_DIAMETER_IN / 2.0);
        double distFromCamera = height / Math.tan(angleRad);
        double distFromIntake = distFromCamera - COLLECTOR_OFFSET_FROM_CAMERA_IN;


        if (distFromIntake <= 0 || distFromCamera > 200) return 999.0;

        return distFromIntake;
    }

    public double calculateStrafingDistance(double ty, double tx) {

        double totalAngleDeg = -(ty + CAMERA_PITCH_DEG);
        if (totalAngleDeg <= 0.1) return 0.0;

        double angleRad = Math.toRadians(totalAngleDeg);
        double txRad = Math.toRadians(tx);
        double tyRad = Math.toRadians(ty);

        double effectiveHeight = CAMERA_HEIGHT_IN - (BALL_DIAMETER_IN / 2.0);


        double strafeDist = effectiveHeight * (Math.cos(tyRad) * Math.tan(txRad)) / Math.sin(angleRad);

        return strafeDist;
    }

    public double distanceFromPixelWidth(double pixelWidth) {
        if (pixelWidth <= 0) return 999.0;
        return (BALL_DIAMETER_IN * FOCAL_LENGTH) / pixelWidth;
    }


    public double findFocalLength(double pixelWidth, double knownDistance) {
        return (pixelWidth * knownDistance) / BALL_DIAMETER_IN;
    }

    public double[] ballToFieldCoords(Pose2d robotPose, double ty, double tx) {
        double heading = robotPose.heading.toDouble();

        double distFromIntake = calculateBallDistance(ty);
        if (distFromIntake >= 999.0) return new double[]{ robotPose.position.x, robotPose.position.y };

        double distFromRobotCenter = distFromIntake + COLLECTOR_OFFSET_FROM_CAMERA_IN + CAMERA_FORWARD_OFFSET_IN;
        double relY = calculateStrafingDistance(ty, tx);

        double fieldX = robotPose.position.x
                + distFromRobotCenter * Math.cos(heading) - relY * Math.sin(heading);
        double fieldY = robotPose.position.y
                + distFromRobotCenter * Math.sin(heading) + relY * Math.cos(heading);

        return new double[]{ fieldX, fieldY };
    }


    // driving + pid helpers

    private void setDrive(double forward, double lateral) {
        robot.drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(forward, lateral), 0)
        );
    }

    private void stopDriving() {
        setDrive(0, 0);
    }

    private void resetPID() {
        lastTx = 0.0;
        lastDirectError = 0.0;
        timerForD.reset();
        timerForForwardD.reset();
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double calculateLateralPower(double error) {
        double dt = Math.max(timerForD.seconds(), 0.001);
        double derivative = (error - lastTx) / dt;
        lastTx = error;
        timerForD.reset();

        return clamp(
                -(LATERAL_KP * error + LATERAL_KD * derivative),
                -MAX_LATERAL_POWER, MAX_LATERAL_POWER
        );
    }


    public double calculateDirectDistance(double ty, double tx) {
        double forwardDist = calculateBallDistance(ty);
        if (forwardDist >= 999.0) return 999.0;

        double strafeDist = calculateStrafingDistance(ty, tx);

        return Math.hypot(forwardDist, strafeDist);
    }
    private double calculateForwardPower(double directDist) {

        double error = directDist - FORWARD_DEADBAND_IN;

        double dt = Math.max(timerForForwardD.seconds(), 0.001);
        double derivative = (error - lastDirectError) / dt;
        lastDirectError = error;
        timerForForwardD.reset();


        return clamp(
                (FORWARD_KP * error) + (FORWARD_KD * derivative),
                0, MAX_FORWARD_POWER
        );
    }



    // MOTIF STUFF


    public static class BallResult {
        public final double tx; // horizontal angle offset (degrees)
        public final double ty; // vertical angle offset (degrees)
        public final double distance; // estimated distance (inches)
        public final double radius; // pixel radius
        public final int numBalls; // total balls seen this frame

        public BallResult(double tx, double ty, double distance, double radius, int numBalls) {
            this.tx = tx;
            this.ty = ty;
            this.distance = distance;
            this.radius = radius;
            this.numBalls = numBalls;
        }
    }

    public List<double[]> getAllBallFieldPositions() {
        Pose2d pose = robot.drive.localizer.getPose();
        List<double[]> positions = new ArrayList<>();
        for (BallResult ball : getAllBalls()) {
            positions.add(ballToFieldCoords(pose, ball.ty, ball.tx));
        }
        return positions;
    }


    // APRIL TAG STUFF =====================================

    public void requestObeliskColors() {
        if (currentLLState == LLState.BALL_PIPELINE) {
            limelight.pipelineSwitch(apriltagPipeline);
            pipelineTimer.reset();
            currentLLState = LLState.SWITCHING_TO_TAG_PIPELINE;
        }
    }

    private void processAprilTags() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            feducialResult = result.getFiducialResults().get(0).getFiducialId();

            switch(feducialResult){
                case 21:
                    robot.ballTracker.targetMotif = robot.ballTracker.motif3;
                    targetOrder = new ArrayList<>(Arrays.asList("G", "P", "P"));
                    break;
                case 22:
                    robot.ballTracker.targetMotif = robot.ballTracker.motif2;
                    targetOrder = new ArrayList<>(Arrays.asList("P", "G", "P"));
                    break;
                case 23:
                    robot.ballTracker.targetMotif = robot.ballTracker.motif1;
                    targetOrder = new ArrayList<>(Arrays.asList("P", "P", "G"));
                    break;
            }
            telemetry.addData("Successfully read Tag ID", feducialResult);
        } else {
            feducialResult = -1;
        }

        limelight.pipelineSwitch(snapscriptPipeline);
        pipelineTimer.reset();
        currentLLState = LLState.SWITCHING_TO_BALL_PIPELINE;
    }
    public int motifRotation(int num) {

        /*
        coutner clockwin, B1 is collector spot
         */

        String B1 = "";
        String B2 = "";
        String B3 = "";

        if (num == 1 || num == 0) {
            B1 = "P";
            B2 = "P";
            B3 = "G";
        } else if ( num == 2 ){
            B1 = "P";
            B2 = "G";
            B3 = "P";
        }else if ( num == 3 ){
            B1 = "G";
            B2 = "P";
            B3 = "P";
        }
        List<String> order1 = new ArrayList<>(Arrays.asList(B3, B1, B2)); // TODO: CHECK
        // G, P, P

//        List<String> order2 = new ArrayList<>(order1);
//        Collections.rotate(order2, 1);
        List<String> order2 = new ArrayList<>(Arrays.asList(B1, B2, B3));
        // P, P, G

//        List<String> order3 = new ArrayList<>(order2);
//        Collections.rotate(order3, 1);
        List<String> order3 = new ArrayList<>(Arrays.asList(B2, B3, B1));
        // P, G, P
//        throw new RuntimeException("target order: " + Arrays.toString(this.targetOrder.toArray()) + ", order1: " + Arrays.toString(order1.toArray()) + ", order2: " + Arrays.toString(order2.toArray()) + ", order3: " + Arrays.toString(order3.toArray()));

        if (order1.equals(this.targetOrder)) {
            return 0;
        } else if (order2.equals(this.targetOrder)) {
            return 1024 /3;
        } else if  (order3.equals(this.targetOrder)){
            return (2 * 1024) /3;
        }
        return 1024;
    }

    public Action resetOdoWLime() {
        return new Action() {
            boolean requested = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!requested) {
                    limelight.pipelineSwitch(apriltagPipeline);
                    pipelineTimer.reset();
                    requested = true;
                    return true;
                }

                if (pipelineTimer.milliseconds() < PIPELINE_SWITCH_DELAY_MS) {
                    return true;
                }

                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid() && result.getTa() > 0.5) {
                    Pose3D botpose = result.getBotpose();

                    if (botpose != null) {
                        double x = botpose.getPosition().x * 39.3701; // in inches
                        double y = botpose.getPosition().y * 39.3701;
                        double headingRad = Math.toRadians(botpose.getOrientation().getYaw());

                        Pose2d newPose = new Pose2d(x, y, headingRad);
                        robot.drive.localizer.setPose(newPose);

                        telemetry.addLine("LIMELIGHT RESETTED");
                    }
                }

                limelight.pipelineSwitch(snapscriptPipeline);
                pipelineTimer.reset();
                currentLLState = LLState.SWITCHING_TO_BALL_PIPELINE;

                return false;
            }
        };
    }

}