package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.util.Range;

public class ShooterHoodLookup {
    public static double minDist = 45.3, maxDist = 97.5;
    private InterpLUT shooterLookup;
    private InterpLUT hoodLookup;

    public ShooterHoodLookup() {
        shooterLookup = new InterpLUT();
        hoodLookup = new InterpLUT();

        shooterLookup.add(45.2548, 1220);
        hoodLookup.add(45.2548, 0.62);

        shooterLookup.add(55, 1240);
        hoodLookup.add(55, 0.60);

        shooterLookup.add(60, 1255);
        hoodLookup.add(60, 0.595);

        shooterLookup.add(66.759, 1300); // may need to be changed
        hoodLookup.add(66.759, 0.59);

        shooterLookup.add(79.5656, 1350);
        hoodLookup.add(79.5656, 0.57);

        shooterLookup.add(97.5807, 1450);
        hoodLookup.add(97.5807, 0.55);

        shooterLookup.createLUT();
        hoodLookup.createLUT();
    }

    public double getShooterSpeed(double dist) {
        dist = Range.clip(dist, minDist, maxDist);
        return shooterLookup.get(dist);
    }
    public double getHoodPosition(double dist) {
        dist = Range.clip(dist, minDist, maxDist);
        return hoodLookup.get(dist);
    }
}
