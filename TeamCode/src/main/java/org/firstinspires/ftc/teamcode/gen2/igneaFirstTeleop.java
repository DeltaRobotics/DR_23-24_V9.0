package org.firstinspires.ftc.teamcode.gen2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHardware;

@TeleOp(name="igneaFirstTeleop")
//@Disabled

public class igneaFirstTeleop extends LinearOpMode{

    public Servo finger = null;
    public Servo elbow = null;
    public Servo wrist = null;
    public Servo shoulder = null;
    public Servo intakeServo = null;


    public DcMotor intake = null;
    public DcMotor slidesL = null;
    public DcMotor slidesR = null;

    public boolean slidesRasied = false;
    public int stickPosition = 0;
    /*
    * 0 - hopper
    * 1 - top - bottom
    * 2 - top left - bottom right
    * 3 - top right - bottom left
    * 4 - first left - second right
    * 5 - first right - second left
    * */

    public int slideEncoder = 0;

    public boolean slidesUp = false;
    public boolean slidesDown = true;
    public boolean servoAdjust = true;

    public double servoVariable = 0;
    public double ShoulderPos = 0.5;

    public boolean stickDropping = false;

    public boolean unoPixo = false;

    public boolean rightBumper = true;
    public boolean leftBumper = true;

    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        ElapsedTime servoTime = new ElapsedTime();

        finger = hardwareMap.servo.get("finger");
        elbow = hardwareMap.servo.get("elbow");
        wrist = hardwareMap.servo.get("wrist");
        shoulder = hardwareMap.servo.get("shoulder");
        intakeServo = hardwareMap.servo.get("intakeServo");

        intake = hardwareMap.dcMotor.get("intake");
        slidesL = hardwareMap.dcMotor.get("slidesL");
        slidesR = hardwareMap.dcMotor.get("slidesR");

        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        //0.95 for grabbing from intake

        while(!isStarted() && !isStopRequested()){

            shoulder.setPosition(.1);
            wrist.setPosition(.82);
            elbow.setPosition(.48);
            finger.setPosition(.75);
            intakeServo.setPosition(0.78);

            //intakeServo intake pos = .55
            //max slide distance = 730

            //servoVariable = servoTime.seconds();
            //if (servoTime.seconds() == servoVariable + 1){

            //}
        }

        while (opModeIsActive()) {

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.75);

            intakeServo.setPosition(.58);
            if(gamepad2.a){
                shoulder.setPosition(shoulder.getPosition()+.01);
            }
            if(gamepad2.b){
                shoulder.setPosition(shoulder.getPosition()-.01);
            }
            if(gamepad2.x){
                wrist.setPosition(wrist.getPosition()+.01);
            }
            if(gamepad2.y){
                wrist.setPosition(wrist.getPosition()-.01);
            }

            //servo stick set positions
            if(gamepad1.a){
                //placing
                wrist.setPosition(.35);
                elbow.setPosition(.68);
                shoulder.setPosition(.55);
            }
            if(gamepad1.b){
                //collecting
                wrist.setPosition(.21);
                elbow.setPosition(.68);
                shoulder.setPosition(.7);
            }
            if(gamepad1.x){
                shoulder.setPosition(.65);
            }
            telemetry.addData("shoulder",shoulder.getPosition());
            telemetry.addData("wrist",wrist.getPosition());
            telemetry.addData("elbow",elbow.getPosition());
            telemetry.addData("finger",finger.getPosition());

            if(gamepad1.left_bumper && leftBumper){
                finger.setPosition(finger.getPosition()+.01);
                unoPixo = false;
                leftBumper = false;
            }
            if(gamepad1.right_bumper && rightBumper){
                //2 pixel code .5
                finger.setPosition(finger.getPosition()-.01);

                unoPixo = true;
                rightBumper = false;
            }
            if(gamepad1.right_bumper && unoPixo && rightBumper){
                //1 pixel code = .3
                finger.setPosition(finger.getPosition()-.01);

                rightBumper = false;
            }

            if (!gamepad1.right_bumper && !rightBumper){
                rightBumper = true;
            }
            if(!gamepad1.left_bumper && !leftBumper){
                leftBumper = true;
            }


            //intake
            if(gamepad1.right_trigger > 0.5){
                intake.setPower(0.65);
            }
            else if(gamepad1.left_trigger > 0.5){
                intake.setPower(-0.55);
            }
            else {
                intake.setPower(0);
            }
            //if(intake.getPower() != 0 && (gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5)){
            //    intake.setPower(0);
            //}

            //servo fine adjust
            //if (gamepad1.back && servoAdjust || servoVariable > 0){
//
            //    if(!stickDropping) {
            //        servoVariable = servoTime.milliseconds();
            //        stickDropping = true;
            //    }
//
            //    ShoulderPos = .9;
            //    if (servoVariable + 200 < servoTime.milliseconds()){
            //        ShoulderPos = .91;
            //    }
            //    if (servoVariable + 400 < servoTime.milliseconds()){
            //        ShoulderPos = .92;
            //    }
            //    if (servoVariable + 600 < servoTime.milliseconds()){
            //        ShoulderPos = .93;
            //    }
            //    if (servoVariable + 800 < servoTime.milliseconds()){
            //        ShoulderPos = .94;
            //    }
            //    if (servoVariable + 1000 < servoTime.milliseconds()){
            //        ShoulderPos = .95;
            //        servoVariable = 0;
            //        stickDropping = false;
            //    }
//
            //    shoulder.setPosition(ShoulderPos);
            //    servoAdjust = false;
            //}
            //else if (!gamepad1.back && !servoAdjust){
            //    servoAdjust = true;
            //}



            //slides
            if(gamepad1.dpad_up){
                slidesRasied = true;
                //stacks high
                slideEncoder = 650;

                stick_mid();
            }
            if(gamepad1.dpad_right){
                slidesRasied = true;
                //slides mid
                slideEncoder = 400;

                stick_mid();
            }
            if(gamepad1.dpad_down){
                slidesRasied = true;
                //slides low
                slideEncoder = 150;

                stick_mid();
            }
            if(gamepad1.dpad_left){
                slidesRasied = false;
                //slides retracted

                stickPosition = 0;
                stick_hopper();

            }

            /**
            //manipulator controls
            if(slidesRasied){
                if(stickPosition == 0){
                    telemetry.addData("Claw position", "hopper");

                }
                if(stickPosition == 1){
                    telemetry.addData("Claw position", "top - bottom");

                }
                if(stickPosition == 2){
                    telemetry.addData("Claw position", "top left - bottom right");

                }
                if(stickPosition == 3){
                    telemetry.addData("Claw position", "top right - bottom left");

                }
                if(stickPosition == 4){
                    telemetry.addData("Claw position", "first left - second right");

                }
                if(stickPosition == 5){
                    telemetry.addData("Claw position", "first right - second left");

                }
            } else {
                if(stickPosition == 0){
                    telemetry.addData("Claw position", "hopper");
                }
                if(stickPosition == 1){
                    telemetry.addData("Claw position", "top - bottom");
                }
                if(stickPosition == 2){
                    telemetry.addData("Claw position", "top left - bottom right");
                }
                if(stickPosition == 3){
                    telemetry.addData("Claw position", "top right - bottom left");
                }
                if(stickPosition == 4){
                    telemetry.addData("Claw position", "first left - second right");
                }
                if(stickPosition == 5){
                    telemetry.addData("Claw position", "first right - second left");
                }

            }

            if(gamepad2.a && stickPosition <= 5){
                stickPosition += 1;
            }
            if(gamepad2.b && stickPosition >= 0){
                stickPosition -= 1;
            }

            //larger stick adjust
            if(gamepad2.dpad_up){
                stick_up();
            }
            if(gamepad2.dpad_right){
                stick_mid();
            }
            if(gamepad2.dpad_down){
                stick_bottom();
            }
             **/

            if(slidesR.getCurrentPosition() < 20 && slideEncoder < 20){
                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(0);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(0);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            telemetry.addData("x",robot.GlobalX);
            telemetry.addData("y",robot.GlobalY);
            telemetry.addData("heading",Math.toDegrees(robot.GlobalHeading));
            telemetry.addData("slide Encoder value", slideEncoder);
            telemetry.update();
        }
    }
    public void stick_up(){}
    public void stick_mid(){}
    public void stick_bottom(){}
    public void stick_hopper(){}
}