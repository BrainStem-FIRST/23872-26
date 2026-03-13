package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@Config
public class Constants {
        public static class ShooterConstants {

            // PID Constants
            // TODO: Tune for built in PID
            public  double kP_ONE = 0.0032;
            public double kP_TWO = 0.0038;
            public  double kI = 0.0;
            public  double kD = 0.0000003;
            public  double kV1 = 0.00042;

            // Vel Targets (ticks per second)
            public  double AUTO_VEL = 300;
            public  double FAR_SHOOT_VEL = 1920;
            public  double CLOSE_SHOOT_VEL = 1220;

            // Limits
            public  double MAX_TICKS_PER_SEC = 2300;
            public   double IDLE_POWER = 0.2;
            public   double MAX_POWER = 0.99;

        }
        
        public static ShooterConstants shooterConstants = new ShooterConstants();



        public static class SpindexerConstants {
            // TODO: Update ticks for new encoder

            // PID Constants
            public double INDEXER_KP = -0.0036;
            public double INDEXER_KI = 0.0;
            public double INDEXER_KD = 0.00005;
            public double INDEXER_KF = 0.035;

            // Position Constants (ticks)
            public int TICKS_120 = 1024/3;
            public int TICKS_60 = 48* (1024/288);
            public int TICKS_360 = 1024;
            public double ERROR_THRESHOLD = 5;

            // Limits
            public double MAX_POWER = 0.99;

            // Anti-Jam Detection Constants
            public double MIN_VEL_TO_START_CHECKING = 5;
            public double MIN_ERROR_TO_START_CHECKING = 30;
        }
        public static SpindexerConstants spindexerConstants = new SpindexerConstants();

        public static class CollectorConstants {
            // Power Constants
            public static double INTAKE_VELOCITY = 3300;
            public static double EXTAKE_VELOCITY = -800;
            public static double AUTO_VELOCITY = 3300; // ).77
            public static final double OFF_POWER = 0.0;
        }

//        public static class FingerConstants {
//
//            // Servo PWM Range
//            public static final int DOWN_PWM = 837;
//            public static final int UP_PWM = 1258;
//
//            // Servo Position
//            public static final double DOWN_POSITION = 0.07;
//            public static final double UP_POSITION = 0.99;
//
//            // Timing Constants (seconds)
//            public static final double UP_TIME = 0.3;
//            public static final double DOWN_TIME = 0.3;
//        }

         public static final class RampConstants {

        // Servo PWM Range
        public int DOWN_PWM = 600;
        public int UP_PWM = 970;

        // Servo Position
        public double DOWN_POSITION = 0.99; // change
        public double UP_POSITION = 0.4; // change
    }


    public static RampConstants rampConstants = new RampConstants();

    public static final class ParkConstants {

        // Servo PWM Range
        public int DOWN_PWM = 1050;
        public int UP_PWM = 1950;

        // Servo Position
        public double DOWN_POSITION = 0.01; // change
        public double UP_POSITION = 0.99; // change
    }

    public static ParkConstants parkConstants = new ParkConstants();


    public static final class PivotConstants {

        // Servo Position


        // On Control hub
        public static final int LEFT_DOWN_POS = 400; // change
        public static final int LefT_UP_POS = 2400; // change


        // On expansion hub
        public static final int RIGHT_DOWN_POS = 2400; // change
        public static final int RIGHT_UP_POS = 1; // change
    }

    public static PivotConstants pivotConstants = new PivotConstants();

    public static class DriveConstants {

            public static double ALIGNMENT_KP = 0.4;
            public static double ALIGNMENT_KI = 0.0;
            public static double ALIGNMENT_KD = 0.00001;

        }
}
