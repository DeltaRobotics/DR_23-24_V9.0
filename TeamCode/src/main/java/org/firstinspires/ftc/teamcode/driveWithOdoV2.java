package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="driveWithOdoV2")
//@Disabled

public class driveWithOdoV2 extends LinearOpMode{

    int slideEncoder = 20;
    double slidePower = .5;

    double wristPower = .455;

    int loopNumber = 0;
    int oldLoop = 0;
    boolean clawOpen = true;



    public void runOpMode() throws InterruptedException {


        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        robot.launcher.setPosition(0.75);
        robot.clawL.setPosition(0);
        robot.clawR.setPosition(1);
        robot.wrist.setPosition(wristPower);




        waitForStart();

        while (opModeIsActive()) {

            robot.refresh(robot.odometers);

            //robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 0.75);//Joseph
            robot.mecanumDrive(gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, 0.75);//Nolan

            //drone
            if(gamepad1.right_trigger > .5 && gamepad1.left_trigger > .5){
                //launch drone
                slideEncoder = 0;
                if(robot.slidesL.getCurrentPosition() < 100) {
                    robot.launcher.setPosition(0.6);
                }
            }
            if(gamepad1.dpad_right){
                //reload
                robot.launcher.setPosition(0.75);

            }
            //claw
            if(gamepad1.right_bumper){
                //open
                robot.clawL.setPosition(.1);
                robot.clawR.setPosition(.7);
                clawOpen = true;
                oldLoop = loopNumber;
            }
            if(gamepad1.left_bumper){
                //close
                robot.clawL.setPosition(0);
                robot.clawR.setPosition(1);
                clawOpen = false;
                oldLoop = loopNumber;
            }
            if(slideEncoder < 900 && (loopNumber == oldLoop + 20 && !clawOpen)){
                //close
                wristPower = .55;
            }
            else if(slideEncoder < 900 && (loopNumber == oldLoop + 20 && clawOpen)){
                //open
                wristPower = .4;
            }



            if(gamepad1.a){
                //slides down
                slideEncoder = 400;
                wristPower = .55;
            }
            else if(gamepad1.b){
                //slides low
                slideEncoder = 1000;
                wristPower = .55;
            }
            else if(gamepad1.y){
                //slides mid
                slideEncoder = 2000;
                wristPower = .55;
            }
            else if(gamepad1.x){
                //slides top
                slideEncoder = 3400;
                wristPower = .55;
            }

            //changes the power if going up or down
            if(slideEncoder > robot.slidesL.getCurrentPosition()){
                slidePower = 1;
            }
            else{
                slidePower = .5;
            }

            //wrist fine adjust
            if(gamepad1.dpad_up){
                wristPower += 0.005;
            }
            else if(gamepad1.dpad_down){
                wristPower -= 0.005;
            }

            robot.wrist.setPosition(wristPower);

            robot.slidesR.setTargetPosition(slideEncoder);
            robot.slidesR.setPower(slidePower);
            robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.slidesL.setTargetPosition(slideEncoder);
            robot.slidesL.setPower(slidePower);
            robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            loopNumber++;


            telemetry.addData("x",robot.GlobalX);
            telemetry.addData("y",robot.GlobalY);
            telemetry.addData("heading",robot.GlobalHeading);
            telemetry.addData("left encoder",robot.odometers[0].getCurrentPosition());
            telemetry.addData("right encoder",robot.odometers[1].getCurrentPosition());
            telemetry.addData("perp encoder",robot.odometers[2].getCurrentPosition());
            telemetry.addData("slide encoder",slideEncoder);
            telemetry.addData("servo", robot.wrist.getPosition());
            telemetry.update();
        }
    }
}