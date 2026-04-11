package org.firstinspires.ftc.teamcode.auto.helping_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utils.BallSensor;

@Disabled
@TeleOp(name="Ball Sensor Test", group="Test")
public class BallSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        BallSensor ballSensor = new BallSensor(hardwareMap);

        telemetry.addLine("ready to scan");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // beam break state
            boolean ballPresent = !ballSensor.beamBreak.getState();

            // run detection logic
            String detected = ballSensor.detectColor();

            // telemetry output
            telemetry.addData("Ball Present", ballPresent ? "YES" : "NO");
            telemetry.addData("Detected Color", detected);

            // color sensor min and max thresholds
            telemetry.addLine("raw percentages");
            telemetry.addData("Red %", "%.3f", ballSensor.rPercent);
            telemetry.addData("Green %", "%.3f", ballSensor.gPercent);
            telemetry.addData("Blue %", "%.3f", ballSensor.bPercent);

            // led state
            telemetry.addData("LED Position", ballSensor.ledLight.getPosition());

            telemetry.update();
        }
    }
}