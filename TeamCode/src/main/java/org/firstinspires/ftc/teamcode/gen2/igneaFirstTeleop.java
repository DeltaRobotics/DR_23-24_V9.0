package org.firstinspires.ftc.teamcode.gen2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotHardware;

@TeleOp(name="igneaFirstTeleop")
//@Disabled

public class igneaFirstTeleop extends LinearOpMode{

    public Servo Finger = null;
    public DcMotor intake = null;

    public void runOpMode() throws InterruptedException {

        //robotHardware robot = new robotHardware(hardwareMap);

        Finger = hardwareMap.servo.get("Finger");
        intake = hardwareMap.dcMotor.get("intake");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a){
                Finger.setPosition(0.15);
            }
            if(gamepad1.b){
                Finger.setPosition(.49);
            }
            if(gamepad1.x){
                Finger.setPosition(1);
            }

            if(gamepad1.right_bumper){
                intake.setPower(0.5);
            }
            if(gamepad1.left_bumper){
                intake.setPower(0);
            }

        }
    }
}