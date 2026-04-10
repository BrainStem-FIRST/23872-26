package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;

import java.util.List;

@Config
public class LimelightForFunv2 implements Component {
    public int pipeline = 0;
    public Limelight3A limelight;

    public List<String> targetOrder;




    BrainSTEMRobot robot;
    public static int feducialResult = -1;

    private Telemetry telemetry;

    public LimelightForFunv2(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.robot = robot;

        limelight.pipelineSwitch(pipeline);
        limelight.start();


    }


    @Override
    public void update() {

        // updating limelight

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double[] customResult = result.getPythonOutput();

            if (customResult.length > 0) {

                double xVal = customResult[0];
                double yVal = customResult[1];
                double radius = customResult[2];
                double tx = customResult[3];
                double ty = customResult[4];
                double pixelWidth = customResult[5];
                double distance = findDistance(pixelWidth);
                boolean tv = customResult[6] == 1;

                double[] drivePowers = SimplePIDDrive.calculate(tx, distance, tv);

                double headingPower = drivePowers[1];
                double forwardPower = drivePowers[0];

                robot.setLeftPower(forwardPower - headingPower);
                robot.setLeftPower(forwardPower + headingPower);

                telemetry.addData("x val", xVal);
                telemetry.addData("y val", yVal);
            }

        }
    }



    public void autoCollect() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            double basicDistance = Math.sqrt(
                   Math.pow(tx,2) + Math.pow(ty,2)
            );

            double headingPower = tx * 0.03;

            double drivePower = (25.0 - ta) * 0.04;

            if (ta > 15.0) {
                robot.collector.on();
            }



        } else {

        }
    }

    public double findDistance(double pixelWidth) {
        return (double) (4.90 * findFocalLength(300, 10))/pixelWidth;
    }

    public double findFocalLength(int fixedPixelWidth, double distance) {
        return (double) (fixedPixelWidth * distance)/ 4.90;
    }


    @Override
    public void reset() {

    }
    @Override
    public String test() {return null;}

}