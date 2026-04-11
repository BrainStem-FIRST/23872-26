package org.firstinspires.ftc.teamcode.auto.helping_opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utils.misc.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.PIDController;


@TeleOp
@Disabled
public class SpindexerTuning extends OpMode {

    private GamepadTracker gp1;
    private GamepadTracker gp2;

    public static double kP = 0; // tune this
    public static double kI = 0.0;
    public static double kD = 0.0; // tune if needed
    public static double kF = 0.0; // dont touch

    public double highPosition = 96;
    public double lowPosition = 0;
    double currentTarget = lowPosition;

    double[] stepSizes = {100.0, 10.0, 1.0, 0.1, 0.01, 0.001};

    int stepIndex = 2;
    int pidParam = 0; // 0 = P, 1 = I, 2 = D

    public DcMotorEx spinnerMotor;

    private PIDFCoefficients spinnerPID;
    private PIDController spinnerPIDController;

    public void init() {
        spinnerMotor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");

        spinnerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinnerPIDController = new PIDController(kP, kI, kD);


//        gp1 = new GamepadTracker(gamepad1);
//        gp2 = new GamepadTracker(gamepad2);

        spinnerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        spinnerPID = new PIDFCoefficients(kP, kI, kD, kF);



//        spinnerMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, spinnerPID);

        telemetry.addLine("init complete");
        telemetry.addLine("Y: Toggle target position");
        telemetry.addLine("B: Change step size");
        telemetry.addLine("A: Cycle PID parameter (P/I/D)");
        telemetry.addLine("D-Pad L/R: Decrease/Increase selected parameter");
    }

    public void loop() {

        if (gamepad1.y) {
            currentTarget = (currentTarget == highPosition) ? lowPosition : highPosition;
        }

        if (gamepad1.xWasPressed()) {
            telemetry.addData("X Button Pressed", "true");
            currentTarget += 80;
        }


        if (gamepad1.dpadUpWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadDownWasPressed()) {
            pidParam = (pidParam + 1) % 3;
        }

        if (gamepad1.aWasPressed()) {
            switch (pidParam) {
                case 0: kP -= stepSizes[stepIndex]; break;
                case 1: kI -= stepSizes[stepIndex]; break;
                case 2: kD -= stepSizes[stepIndex]; break;
            }
            telemetry.addData("X Button Pressed", "false");
        }

        if (gamepad1.bWasPressed()) {
            switch (pidParam) {
                case 0: kP += stepSizes[stepIndex]; break;
                case 1: kI += stepSizes[stepIndex]; break;
                case 2: kD += stepSizes[stepIndex]; break;
            }
        }


        kP = Math.max(0, kP);
        kI = Math.max(0, kI);
        kD = Math.max(0, kD);

        spinnerPIDController.setTarget(currentTarget);
        spinnerPIDController.setPIDValues(kP, kI, kD);


        int currentPosition = spinnerMotor.getCurrentPosition();
        double error = currentTarget - currentPosition;

        if (Math.abs(error) <= 3){
            spinnerMotor.setPower(0);
        } else {
            spinnerMotor.setPower(spinnerPIDController.update(currentPosition));
        }
//        spinnerMotor.setPower(-spinnerPIDController.update(currentPosition));

        telemetry.addData("Target Position", "%.0f", currentTarget);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Is Busy", spinnerMotor.isBusy());
        telemetry.addLine("--------------------");

        String[] paramNames = {"P", "I", "D"};
        double[] paramValues = {kP, kI, kD};

        telemetry.addData("Tuning Parameter", paramNames[pidParam] + " (A Button to cycle)");
        telemetry.addData("P", "%.4f %s", kP, pidParam == 0 ? "<--" : "");
        telemetry.addData("I", "%.4f %s", kI, pidParam == 1 ? "<--" : "");
        telemetry.addData("D", "%.4f %s", kD, pidParam == 2 ? "<--" : "");
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();

//        gp1.update();
//        gp2.update();
    }
}