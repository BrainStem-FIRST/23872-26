package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;
@Config
public class Parking implements Component {

    public static boolean activateLeft = true, activateRight = true;
    private Telemetry telemetry;
    public ServoImplEx parkServo;
    public ServoImplEx parkServo2;

    public ParkState parkState;

    public double targetPosition;

    public HardwareMap map;
    public enum ParkState {
        DOWN,
        UP
    }

    public Parking(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.parkState = ParkState.DOWN;

        parkServo = map.get(ServoImplEx.class, "leftPark"); // Control
        parkServo2 = map.get(ServoImplEx.class, "rightPark"); // Expansion
        parkServo.setPwmRange(new PwmControl.PwmRange(950, 2100)); // CHANGE\
        parkServo2.setPwmRange(new PwmControl.PwmRange(950, 2100)); // CHANGE
        targetPosition = Constants.parkConstants.DOWN_POSITION;
    }


    @Override
    public void reset() {}

    public void setTargetPosition() {
        if (activateLeft) parkServo.setPosition(targetPosition);

        if (activateRight) parkServo2.setPosition(targetPosition);

    }

    @Override
    public void update() {
        switch (parkState) {
            case DOWN:
                targetPosition = 0.01;
                break;
            case UP:
                targetPosition = 0.99;
                break;
        }
        setTargetPosition();

        telemetry.addData("Parking State", parkState);

    }

    public void setParkUp() { parkState = ParkState.UP;}
    public void setParkDown() { parkState = ParkState.DOWN;};

    public ParkState checkParkState() {
        return parkState;
    }
    public boolean isParkUp() {
        return parkState == ParkState.UP.UP;
    }

    @Override
    public String test() {
        return null;
    }

}



