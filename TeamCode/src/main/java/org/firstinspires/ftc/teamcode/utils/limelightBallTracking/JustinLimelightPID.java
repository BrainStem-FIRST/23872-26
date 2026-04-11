package org.firstinspires.ftc.teamcode.utils.limelightBallTracking;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.utils.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

public class JustinLimelightPID {
    public static Action alignRobotLateral(MecanumDrive drive, Limelight limelight) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                LLResult result = limelight.limelight.getLatestResult();
                double[] pythonOutputs = result.getPythonOutput();
                double tx = pythonOutputs[3];
                
                if (Math.abs(tx) < 5) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    return false;
                }

                double lateralPower = tx / 15.;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -lateralPower), 0));
                return true;
            }
        };
    }
    public static Action driveThroughBall(MecanumDrive drive) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
    }
    public static Action fullLimelightCollect(MecanumDrive drive, Limelight limelight) {
        return new SequentialAction(
                alignRobotLateral(drive, limelight),
                driveThroughBall(drive)
        );
    }
}
