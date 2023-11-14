package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="slidesTest1")
@Disabled

public class slidesTest1 extends LinearOpMode{

    int slideEncoder = 0;
    double slidePower = .5;



    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        robot.launcher.setPosition(0.85);

        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 0.75);

            if(gamepad1.right_bumper && gamepad1.left_bumper){
                //launch drone
                robot.launcher.setPosition(0.65);
            }
            if(gamepad1.a){
                //reload
                robot.launcher.setPosition(0.85);
            }

            //slides max is around 3500 encoder tics

            if(gamepad1.a){
                //slides down
                slidePower = .5;
                slideEncoder = 25;
            }
            else if(gamepad1.b){
                //slides max
                slidePower = .7;
                slideEncoder = 3500;
            }

            robot.slidesR.setTargetPosition(slideEncoder);
            robot.slidesR.setPower(slidePower);
            robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.slidesL.setTargetPosition(slideEncoder);
            robot.slidesL.setPower(slidePower);
            robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            telemetry.addData("slide encoder",slideEncoder);
            telemetry.addData("servo", robot.launcher.getPosition());
            telemetry.update();
        }
    }
}