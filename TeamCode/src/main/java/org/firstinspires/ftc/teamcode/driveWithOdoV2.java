package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="driveWithOdoV2")
//@Disabled

public class driveWithOdoV2 extends LinearOpMode{

    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        robot.launcher.setPosition(0.85);



        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 0.75);

            if(gamepad1.right_bumper && gamepad1.left_bumper){
                //launch drone
                robot.launcher.setPosition(0.65);
            }
            if(gamepad1.a){
                //reload
                robot.launcher.setPosition(0.85);
            }



            telemetry.addData("servo", robot.launcher.getPosition());
            telemetry.update();
        }
    }
}