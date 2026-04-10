package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.BallTrackerNew;
import org.firstinspires.ftc.teamcode.utils.Component;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Limelight implements Component {
    public int pipeline = 0;
    public Limelight3A limelight;

    public BallTrackerNew ballTrackerNew;

    public List<String> targetOrder;


    BrainSTEMRobot robot;
    public static int feducialResult = -1;

    private Telemetry telemetry;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.robot = robot;

        limelight.pipelineSwitch(pipeline);
        limelight.start();


       ballTrackerNew = new BallTrackerNew(robot.spindexer);

    }


    @Override
    public void update() {
        ballTrackerNew.update();

    }

    public void updateObeliskColors() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (!result.getFiducialResults().isEmpty())
                feducialResult = result.getFiducialResults().get(0).getFiducialId();

            else {
                feducialResult = -1;
            }
        }
        else {
            feducialResult = -10;
        }

        if (result != null) {
//            telemetry.addData("Fiducial results", result.getFiducialResults());
//            telemetry.addData("Amount", result.getFiducialResults().size());
        }
        switch(feducialResult){
            case(21):
                ballTrackerNew.targetMotif = ballTrackerNew.motif3;
                break;
            case(22):
                ballTrackerNew.targetMotif = ballTrackerNew.motif2;
                break;
            case(23):
                ballTrackerNew.targetMotif = ballTrackerNew.motif1;
                break;
        }
    }

    public void  updateTargetMotif() {

        int tagId = Limelight.feducialResult;

        switch (tagId) {
            case 21:
                targetOrder = new ArrayList<>(Arrays.asList("G", "P", "P"));
                BallTrackerNew.targetMotif = ballTrackerNew.motif3;
                break;
            case 22:
                targetOrder = new ArrayList<>(Arrays.asList("P", "G", "P"));
                BallTrackerNew.targetMotif = ballTrackerNew.motif2;
                break;
            case 23:
                targetOrder = new ArrayList<>(Arrays.asList("P", "P", "G"));
                BallTrackerNew.targetMotif = ballTrackerNew.motif1;
                break;
            default:
                break;
        }

        telemetry.addData("Limelight result", limelight.getLatestResult());
        telemetry.addData("Limelight Tag ID", tagId);
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
//        throw new RuntimeException("target order: " + Arrays.toString(robot.limelight.targetOrder.toArray()) + ", order1: " + Arrays.toString(order1.toArray()) + ", order2: " + Arrays.toString(order2.toArray()) + ", order3: " + Arrays.toString(order3.toArray()));

        if (order1.equals(robot.limelight.targetOrder)) {
            return 0;
        } else if (order2.equals(robot.limelight.targetOrder)) {
            return 1024 /3;
        } else if  (order3.equals(robot.limelight.targetOrder)){
            return (2 * 1024) /3;
        }
        return 1024;
    }

    public Action resetOdoWLime() {
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
                telemetry.addData("New X", x);
                telemetry.addData("New Y", y);
            }
        }
        return null;
    }

    @Override
    public void reset() {

    }
    @Override
    public String test() {return null;}

}