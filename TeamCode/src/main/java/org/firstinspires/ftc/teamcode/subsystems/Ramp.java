package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;
@Config
public class Ramp implements Component {
    private Telemetry telemetry;
    public ServoImplEx rampServo;

    public RampState rampState;

    public double targetPosition;

    public HardwareMap map;
    public enum RampState {
        DOWN,
        UP
    }

    public Ramp(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.rampState = RampState.DOWN;

        rampServo = map.get(ServoImplEx.class, "rampServo");
        rampServo.setPwmRange(new PwmControl.PwmRange(Constants.rampConstants.DOWN_PWM, Constants.rampConstants.UP_PWM));
        targetPosition = Constants.rampConstants.DOWN_POSITION;
    }


    @Override
    public void reset() {}

    public void setTargetPosition() {
        rampServo.setPosition(targetPosition);
    }

    @Override
    public void update() {
        switch (rampState) {
            case DOWN:
                targetPosition = Constants.rampConstants.DOWN_POSITION;
                break;
            case UP:
                targetPosition = Constants.rampConstants.UP_POSITION;
                break;
        }
        setTargetPosition();

        telemetry.addData("Finger State", rampState);
    }

    public void setRampUp() { rampState = RampState.UP;}
    public void setRampDown() { rampState = RampState.DOWN;};

    public RampState checkRampState() {
        return rampState;
    }
    public boolean isRampUp() {
        return rampState == RampState.UP;
    }

    @Override
    public String test() {
        return null;
    }

}



