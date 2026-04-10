package org.firstinspires.ftc.teamcode.utils.pidDrive;

import com.acmerobotics.roadrunner.Pose2d;

public class UtilFunctions {
    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static Pose2d createPose(double[] pose) {
        return new Pose2d(pose[0], pose[1], Math.toRadians(pose[2]));
    }

    public static Pose2d createRedPose(double[] pose) {
        return new Pose2d(pose[0], -pose[1], -Math.toRadians(pose[2]));
    }
}
