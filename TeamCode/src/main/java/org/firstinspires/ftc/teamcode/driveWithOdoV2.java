package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="driveWithOdoV2")
@Disabled

public class driveWithOdoV2 extends LinearOpMode{

    int slideEncoder = 400;
    double slidePower = .5;

    double wristPower = .455;

    int loopNumber = 0;
    int oldLoop = 0;
    boolean clawOpen = true;

    double robotSpeed = .75;

    boolean wristAdjustA = true;
    boolean wristAdjustB = true;

    //non-wheels
    public Servo launcher = null;
    public DcMotor slidesL = null;
    public DcMotor slidesR = null;
    public Servo clawL = null;
    public Servo clawR = null;
    public Servo wrist = null;


    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        launcher.setPosition(0.75);
        clawL.setPosition(0.1);
        clawR.setPosition(0.7);

        waitForStart();

        while (opModeIsActive()) {

            robot.refresh(robot.odometers);

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 0.75);//normal people
            //robot.mecanumDrive(gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, robotSpeed);//Nolan

            //drone
            if(gamepad1.right_trigger > .5 && gamepad1.left_trigger > .5){
                //launch drone
                slideEncoder = 0;
                if(slidesL.getCurrentPosition() < 100) {
                    launcher.setPosition(0.6);
                }
            }
            //if(gamepad1.dpad_right){
            //    //reload
            //    robot.launcher.setPosition(0.75);

            //}
            //claw
            if(gamepad1.right_bumper){
                //open
                clawL.setPosition(.1);
                clawR.setPosition(.7);
                clawOpen = true;
                oldLoop = loopNumber;
            }
            if(gamepad1.left_bumper){
                //close
                clawL.setPosition(0);
                clawR.setPosition(1);
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

            if(gamepad1.dpad_right){
                //slow
                robotSpeed = .75;
            }
            else if(gamepad1.dpad_left){
                //speed
                robotSpeed = .9;
            }



            if(gamepad1.a){
                //slides down
                slideEncoder = 400;
                wristPower = .55;
            }
            else if(gamepad1.b){
                //slides low
                slideEncoder = 1900;
                wristPower = .55;
            }
            else if(gamepad1.x){
                //slides mid
                slideEncoder = 2700;
                wristPower = .55;
            }
            else if(gamepad1.y){
                //slides top
                slideEncoder = 3400;
                wristPower = .55;
            }

            //changes the power if going up or down
            if(slideEncoder > slidesL.getCurrentPosition()){
                slidePower = 1;
            }
            else{
                slidePower = .5;
            }

            //wrist fine adjust
            if(gamepad1.dpad_up && wristAdjustA){
                //1st stack pixel
                wristPower = 0.5;
                wristAdjustA = false;
            }
            else if(!gamepad1.dpad_up && !wristAdjustA){
                wristAdjustA = true;
            }
            if(gamepad1.dpad_down && wristAdjustB){
                //2nd stack pixel
                wristPower = 0.47;
                wristAdjustB = false;
            }
            else if(!gamepad1.dpad_down && !wristAdjustB){
                wristAdjustB = true;
            }
            //.5 and .47 for wrist set pos
            wrist.setPosition(wristPower);

            slidesR.setTargetPosition(slideEncoder);
            slidesR.setPower(slidePower);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slidesL.setTargetPosition(slideEncoder);
            slidesL.setPower(slidePower);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            loopNumber++;


            telemetry.addData("x",robot.GlobalX);
            telemetry.addData("y",robot.GlobalY);
            telemetry.addData("heading",Math.toDegrees(robot.GlobalHeading));
            telemetry.addData("slide encoder",slideEncoder);
            telemetry.addData("servo", wrist.getPosition());
            telemetry.update();
        }
    }
}