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
    public Servo shooter = null;

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
    public double servoVariable2 = 0;
    public double shoulderPos = 0.5;

    public boolean stickDropping = false;

    public boolean unoPixo = false;

    public boolean rightBumper = true;
    public boolean leftBumper = true;

    public double speed = .75;

    public double wristPos = .79;

    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        ElapsedTime servoTime = new ElapsedTime();

        finger = hardwareMap.servo.get("finger");
        elbow = hardwareMap.servo.get("elbow");
        wrist = hardwareMap.servo.get("wrist");
        shoulder = hardwareMap.servo.get("shoulder");
        intakeServo = hardwareMap.servo.get("intakeServo");
        shooter = hardwareMap.servo.get("shooter");

        intake = hardwareMap.dcMotor.get("intake");
        slidesL = hardwareMap.dcMotor.get("slidesL");
        slidesR = hardwareMap.dcMotor.get("slidesR");

        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setPosition(0.75);


        //0.95 for grabbing from intake

        while(!isStarted() && !isStopRequested()){

            shoulder.setPosition(.08);
            wrist.setPosition(.96);
            elbow.setPosition(.77);
            finger.setPosition(.75);
            intakeServo.setPosition(0.78);
            // shoulder = .57
            // wrist = .52 for yellow pixel

            //intakeServo intake pos = .55
            //max slide distance = 730

            //servoVariable = servoTime.seconds();
            //if (servoTime.seconds() == servoVariable + 1){

            //}
        }

        while (opModeIsActive()) {

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed);

            intakeServo.setPosition(.56);

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
            if(gamepad2.dpad_left){
                elbow.setPosition(elbow.getPosition()+.01);
            }
            if(gamepad2.dpad_right){
                elbow.setPosition(elbow.getPosition()-.01);
            }

            //servo stick set positions
            if(gamepad1.b){
                //placing
                wrist.setPosition(.47);
                elbow.setPosition(.77);
                shoulder.setPosition(.55);
                speed = .75;
            }
            if(gamepad1.x || servoVariable > 0){
                //collecting
                speed = .4;
                wrist.setPosition(.41);
                elbow.setPosition(.77);
                shoulderPos = .50;

                if(!stickDropping) {
                    servoVariable = servoTime.milliseconds();
                    stickDropping = true;
                }

                if (servoVariable + 200 < servoTime.milliseconds()){
                    shoulderPos = .62;
                }
                if (servoVariable + 400 < servoTime.milliseconds()){
                    shoulderPos = .66;
                }
                if (servoVariable + 600 < servoTime.milliseconds()){
                    shoulderPos = .68;
                    servoVariable = 0;
                    stickDropping = false;
                }

                shoulder.setPosition(shoulderPos);
            }
            if(gamepad1.y){
                //.72
                // on ground
                shoulder.setPosition(.72);
            }
            if(gamepad1.a){
                //in robot
                shoulder.setPosition(.15);
                wrist.setPosition(.81);
                elbow.setPosition(.77);
            }
            telemetry.addData("shoulder",shoulder.getPosition());
            telemetry.addData("wrist",wrist.getPosition());
            telemetry.addData("elbow",elbow.getPosition());
            telemetry.addData("finger",finger.getPosition());

            if(gamepad1.left_bumper && leftBumper){
                finger.setPosition(.75);
                //change this to finger .75
                unoPixo = false;
                leftBumper = false;
            }
            if(gamepad1.right_bumper && !unoPixo && rightBumper){
                //2 pixel code .5
                finger.setPosition(.5);

                unoPixo = true;
                rightBumper = false;
            }
            if(gamepad1.right_bumper && unoPixo && rightBumper){
                //1 pixel code = .3
                finger.setPosition(.3);

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
                intake.setPower(0.85);
            }
            else if(gamepad1.left_trigger > 0.5){
                intake.setPower(-0.5);
            }
            else {
                intake.setPower(0);
            }

            //servo fine adjust
            //.01 shoulder and .77 for wrist

            if (gamepad1.back || servoVariable2 > 0){
                elbow.setPosition(.81);
                shoulderPos = .09;


                if(!stickDropping) {
                    servoVariable2 = servoTime.milliseconds();
                    stickDropping = true;
                    wrist.setPosition(.85);
                }


                //if (servoVariable2 + 100 < servoTime.milliseconds()){
                //    shoulderPos = .06;
                //}
                //if (servoVariable2 + 200 < servoTime.milliseconds()){
                    //shoulderPos = .03;
                    //wrist.setPosition(.76);
                //}

                if (servoVariable2 + 300 < servoTime.milliseconds()){
                    //shoulderPos = .01;
                    servoVariable2 = 0;
                    stickDropping = false;
                    wrist.setPosition(.76);
                }
                shoulderPos = shoulderPos - ((servoTime.milliseconds() - servoVariable2)*.00026);

                shoulder.setPosition(shoulderPos);
            }
            else if (!gamepad1.back && !servoAdjust){
                servoAdjust = true;
            }



            //slides
            if(gamepad1.dpad_up){
                slidesRasied = true;
                //stacks high
                slideEncoder = 300;

                stick_mid();
            }
            if(gamepad1.dpad_right){
                slidesRasied = true;
                //slides mid
                slideEncoder = 100;

                stick_mid();
            }
            if(gamepad1.dpad_down){
                slidesRasied = true;
                //slides low
                slideEncoder = 0;

                stick_mid();
            }
            if(gamepad1.dpad_left){
                slidesRasied = false;
                //slides retracted

                slideEncoder = 200;
                //stick_hopper();

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

            if(gamepad2.right_bumper && gamepad2.left_bumper){
                shooter.setPosition(0.5);
            }

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
                slidesL.setPower(.8);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.8);
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