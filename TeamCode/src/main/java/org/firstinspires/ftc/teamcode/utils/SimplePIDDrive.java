package org.firstinspires.ftc.teamcode.utils;

public class SimplePIDDrive {

    private static double headingkP = 0.004;
    private static double drivekD = 0.01;
    private static double drivekP = 0.019;

    private static final double STOP_THRESHOLD_TY = 0;

    private static double lastHeadingError = 0;

    public static double[] calculate(double tx, double distance, boolean tv) {
        if (!tv) return new double[]{0, 0};

        double power = 0;
        if ((distance + 1) < 0) {
            double error = distance - STOP_THRESHOLD_TY;
            power = error * drivekP;
        }

        double currentHeadingerror = tx;
        double dH = currentHeadingerror - lastHeadingError; // change in heading

        double headingPower = (currentHeadingerror * headingkP) + (dH * drivekD);

        lastHeadingError = currentHeadingerror;

        power = Math.max(-0.5, Math.min(0.5, power));
        headingPower = Math.max(-0.4, Math.min(0.4, headingPower));

        return new double[]{power, headingPower};
    }
}
