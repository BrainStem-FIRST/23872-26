package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class BallSensor {

    public DigitalChannel beamBreak;
    public NormalizedColorSensor colorSensor;
    public ServoImplEx ledLight; // added for led

    private boolean lastBeamState = true;
    public ElapsedTime timer = new ElapsedTime();
    private boolean isWaitingForColor = false;

    public static double delayTimeMs = 10;
    public static double ballDistance = 5.8;
    public static double ballMinDistance = 0.5;
    public boolean isIndexing = false;

    public static double greenBallMinG = 0.40, greenBallMaxG = 0.53, greenBallMinB = 0.20, greenBallMaxB = 0.38, greenBallMinR = 0.10, greenBallMaxR = 0.25;
    public static double purpleBallMinG = 0.25, purpleBallMaxG = 0.40, purpleBallMinB = 0.2, purpleBallMaxB = 0.6, purpleBallMinR = 0.24, purpleBallMaxR = 0.32;
//

//    public static double greenBallMinG = 0, greenBallMaxG = 0, greenBallMinB = 0, greenBallMaxB = 0, greenBallMinR = 0, greenBallMaxR = 0;
//    public static double purpleBallMinG = 0, purpleBallMaxG = 0, purpleBallMinB = 0, purpleBallMaxB = 0, purpleBallMinR = 0, purpleBallMaxR = 0;


    public static double POS_GREEN = 0.285;  // added (target 1507)
    public static double POS_PURPLE = 0.555; // added (target 1890)
    public static double POS_WHITE = 0.703;  // added (target 2100)



    public double rPercent;
    public double bPercent;
    public double gPercent;
    public double alpha;

    private ElapsedTime settleTimer = new ElapsedTime();
    private boolean waitingForSettle = false;
    public static double settleDelayMs = 0; // wait this long to read color after spindexer

    public BallSensor(HardwareMap hardwareMap) {
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(10);

        ledLight = hardwareMap.get(ServoImplEx.class, "ledLight"); // added for led
        ledLight.setPwmRange(new PwmControl.PwmRange(1102, 2522)); // added for led
    }

    public String scanForNewBall() {
        boolean currentBeamState = !beamBreak.getState(); // false = broken, true = not broken
        String result = null;

        if (!isIndexing && !isWaitingForColor) {
            if (lastBeamState && !currentBeamState) {
                isWaitingForColor = true;
                timer.reset();

            }
        }

        if (isWaitingForColor && timer.milliseconds() > delayTimeMs) {
            //

//               || (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < ballDistance) &&
//                        (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > ballMinDistance)
            isWaitingForColor = false;
            result = detectColor();
        }
        lastBeamState = currentBeamState;
        return result;
    }

//        timer.reset();
//        if (lastBeamState && !currentBeamState) {
//
//            lastBeamState = currentBeamState;
//            if (timer.milliseconds() > delayTimeMs) {
//                 ||
//                (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < ballDistance) &&
//                        (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > ballMinDistance)
//                return detectColor();
//            }
//        }
//
//        lastBeamState = currentBeamState;
//        return null;




    public boolean isDistanceGreaterThanSeven() {
        return (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > 4);
    }



    public String detectColor() {
        NormalizedRGBA colors = ((NormalizedColorSensor)colorSensor).getNormalizedColors();
        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
        alpha = colors.alpha;

        double sum = red + green + blue;

        if (sum < 0.001) { // added
            ledLight.setPosition(POS_WHITE); // added
            return "EMPTY";
        }

        rPercent = red / sum;
        gPercent = green / sum;
        bPercent = blue / sum;

        if (rPercent > greenBallMinR && rPercent < greenBallMaxR &&
                bPercent > greenBallMinB && bPercent < greenBallMaxB &&
                gPercent > greenBallMinG && gPercent < greenBallMaxG) {
            ledLight.setPosition(POS_GREEN); // added for led
            return "GREEN";
        } else if (rPercent > purpleBallMinR && rPercent < purpleBallMaxR &&
                bPercent > purpleBallMinB && bPercent < purpleBallMaxB &&
                gPercent > purpleBallMinG && gPercent < purpleBallMaxG) {
            ledLight.setPosition(POS_PURPLE); // added for led
            return "PURPLE";
        }

        ledLight.setPosition(POS_WHITE); // added for led
        return "EMPTY";
    }

    public String checkColorAfterMovement() {
        if (!waitingForSettle) {
            return null;
        }

        if (settleTimer.milliseconds() > settleDelayMs) {
            waitingForSettle = false;
            return detectColor();
        }

        return null;
    }

    public void setIfIndexerIsMoving(boolean moving) {
        if (this.isIndexing && !moving) {
            this.lastBeamState = true;
            this.isWaitingForColor = false;

            waitingForSettle = true;
            settleTimer.reset();
        }
        this.isIndexing = moving;
    }
}