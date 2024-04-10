package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="CRSTesting")
@Disabled

public class CRSTesting extends LinearOpMode{

    public CRServo finger = null;
    public void runOpMode() throws InterruptedException {

        finger = hardwareMap.crservo.get("finger");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper){
                finger.setPower(1);
            }
            else if (gamepad1.left_bumper){
                finger.setPower(0);
            }

        }
    }
}