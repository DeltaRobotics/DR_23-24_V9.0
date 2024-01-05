package org.firstinspires.ftc.teamcode.gen2;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHardware;

@TeleOp(name="igneaSecondTeleop")
//@Disabled

public class igneaSecondTeleop extends LinearOpMode{

    public CRServo finger = null;
    public Servo shooterAngle = null;
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
    public double shoulderTime = 0;
    public double shoulderGo = 0;

    public boolean stickDropping = false;

    public boolean intakePos = false;
    public boolean outputPos = false;

    public boolean rightBumper = true;
    public boolean leftBumper = true;

    public boolean leftTrigger = false;

    public double speed = .75;

    public double wristPos = .79;

    public boolean buttonB = true;
    public boolean buttonA = true;
    public boolean buttonX = true;
    public boolean buttonY = true;
    public boolean buttonRight = true;
    public boolean buttonLeft = true;

    public double oldOutTime = 0;
    public int newOutTime = 0;



    //RevBlinkinLedDriver blinkinLedDriver;

    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        ElapsedTime outputTime = new ElapsedTime();

        finger = hardwareMap.crservo.get("finger");
        shooterAngle = hardwareMap.servo.get("shooterAngle");
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

        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        shooter.setPosition(0.75);


        //0.95 for grabbing from intake

        while(!isStarted() && !isStopRequested()){

            //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            shooterAngle.setPosition(0.5);
            shooter.setPosition(.54);

            shoulder.setPosition(.4);
            wrist.setPosition(.6);
            //finger.setPower(0);//positive = out
            //intakeServo.setPosition(0.78);
        }

        while (opModeIsActive()) {

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed);

            intakeServo.setPosition(.54);

            if (gamepad1.a && buttonA || stickDropping){
                //intake
                intakePos = true;
                outputPos = false;
                shoulder.setPosition(.58);
                wrist.setPosition(.34);
                buttonA = false;
            }
            else if (gamepad1.b && buttonB){
                //placement
                intakePos = false;
                outputPos = true;
                shoulder.setPosition(.18);
                wrist.setPosition(.8);
                buttonB = false;
            }



            //fine adjust for the arm

            //else if (gamepad1.x && buttonX){
            //    shoulder.setPosition(shoulder.getPosition() + .005);
            //    buttonX = false;
            //}
            //else if (gamepad1.y && buttonY){
            //    shoulder.setPosition(shoulder.getPosition() - .005);
            //    buttonY = false;
            //}


            if (!gamepad1.a && !buttonA){
                buttonA = true;
            }
            else if (!gamepad1.b && !buttonB){
                buttonB = true;
            }

           /* else if (!gamepad1.x && !buttonX){
                buttonX  = true;
            }
            else if (!gamepad1.y && !buttonY){
                buttonY  = true;
            }


            telemetry.addData("shoulder", shoulder.getPosition());
            telemetry.addData("wrist", wrist.getPosition());

            if (gamepad1.right_bumper && buttonRight){
                wrist.setPosition(wrist.getPosition() + .01);
                buttonRight = false;
            }
            else if (gamepad1.left_bumper && buttonLeft){
                wrist.setPosition(wrist.getPosition() - .01);
                buttonLeft = false;
            }
            else if (!gamepad1.right_bumper && !buttonRight){
                buttonRight = true;
            }
            else if (!gamepad1.left_bumper && !buttonLeft){
                buttonLeft = true;
            }
            */

            //intake
            if(gamepad1.right_trigger > 0.5){
                intake.setPower(0.85);
                finger.setPower(-1);
            }
            else if(gamepad1.left_trigger > 0.5){
                if(intakePos){
                    intake.setPower(-0.5);
                } else if(outputPos && !leftTrigger) {
                    finger.setPower(1);
                    oldOutTime = outputTime.milliseconds();
                    leftTrigger = true;
                }
            }
            else {
                intake.setPower(0);
                //finger.setPower(0);
            }
            if(outputPos && leftTrigger && (oldOutTime + 800 < outputTime.milliseconds())){
                finger.setPower(0);
                leftTrigger = false;
            }
            else if(gamepad1.right_trigger < 0.5 && !leftTrigger){
                intake.setPower(0);
                finger.setPower(0);
            }




            //slides
            if(gamepad1.dpad_up){
                slidesRasied = true;
                //stacks high
                slideEncoder = 400;

                stick_mid();
            }
            if(gamepad1.dpad_right){
                slidesRasied = true;
                //slides mid
                slideEncoder = 250;

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

                slideEncoder = 150;
                //stick_hopper();

            }

            //shooter
            if(gamepad2.right_bumper){
                shooterAngle.setPosition(0.58);
            }
            if(gamepad2.right_bumper && gamepad2.left_bumper){
                shooter.setPosition(0.5);
            }

            //setting slide power
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