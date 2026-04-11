package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;
@Config
public class Collector implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx collectorMotor;
    public CollectorState collectorState;
    public enum CollectorState {
        OFF,
        INTAKE,
        EXTAKE,
        AUTO
    }

    public Collector(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        this.collectorState = CollectorState.OFF;

        collectorMotor = map.get(DcMotorEx.class, "collectorMotor");

        collectorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collectorMotor.setVelocityPIDFCoefficients(
                16,
                0.1,
                0,
                14
        );
    }

    @Override
    public void reset() {}

    @Override
    public void update() {
        switch (collectorState) {
            case OFF:
                collectorMotor.setPower(Constants.CollectorConstants.OFF_POWER);
                break;
            case INTAKE:
                collectorMotor.setVelocity(Constants.CollectorConstants.INTAKE_VELOCITY);
                break;
            case EXTAKE:
                collectorMotor.setVelocity(Constants.CollectorConstants.EXTAKE_VELOCITY);
                break;
            case AUTO:
                collectorMotor.setVelocity(Constants.CollectorConstants.AUTO_VELOCITY);
        }
    }

    public void on() {
        collectorState = CollectorState.INTAKE;
    }
    public void off() {
        collectorState = CollectorState.OFF;
    }
    @Override
    public String test() {return null;}


}