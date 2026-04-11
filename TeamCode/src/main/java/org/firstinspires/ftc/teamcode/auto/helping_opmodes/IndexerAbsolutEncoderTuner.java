package org.firstinspires.ftc.teamcode.auto.helping_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.srs.SRSHub;

public class IndexerAbsolutEncoderTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SRSHub.Config config = new SRSHub.Config();

        config.setEncoder(
                6,
                SRSHub.Encoder.PWM
        );

        RobotLog.clearGlobalWarningMsg();

        SRSHub hub = hardwareMap.get(
                SRSHub.class,
                "SRSHub"
        );
        hub.init(config);

        while (opModeIsActive()) {

            hub.update();
            double encoder = hub.readEncoder(6).position;
            telemetry.addData("raw srs hub encoder", encoder);
        }
    }
}
