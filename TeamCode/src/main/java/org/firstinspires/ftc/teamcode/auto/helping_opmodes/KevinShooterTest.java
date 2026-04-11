package org.firstinspires.ftc.teamcode.auto.helping_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name="shooter power test")
public class KevinShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double signFlip = -1;
        DcMotorEx shooterMotorOne = hardwareMap.get(DcMotorEx.class, "shooterMotorOne");
        DcMotorEx shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "shooterMotorTwo");
        double power = 0.5;
        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.dpadUpWasPressed())
                power += 0.1;
            else if(gamepad1.dpadDownWasPressed())
                power -= 0.1;
            if(gamepad1.aWasPressed())
                signFlip *= -1;
            shooterMotorTwo.setPower(power);
            shooterMotorOne.setPower(power * signFlip);

            telemetry.addData("power", power);
            telemetry.addData("sign flip", signFlip);
            telemetry.update();
        }

    }
}
