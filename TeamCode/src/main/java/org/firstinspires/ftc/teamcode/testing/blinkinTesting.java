package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="blinkinTesting")
//@Disabled

public class blinkinTesting extends LinearOpMode{

    RevBlinkinLedDriver blinkinLedDriver;

    public void runOpMode() throws InterruptedException {

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            if (gamepad1.b){
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
            if (gamepad1.x){
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            if (gamepad1.y){
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
            }


        }
    }
}