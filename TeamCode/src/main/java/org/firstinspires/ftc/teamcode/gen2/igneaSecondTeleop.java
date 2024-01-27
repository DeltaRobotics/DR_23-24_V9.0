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

    public boolean intakePos = true;
    public boolean outputPos = false;

    public boolean rightBumper = true;
    public boolean leftBumper = true;

    public boolean leftTrigger = false;

    public double speed = .75;

    public double wristPos = .79;
    public double intakePos2 = .53;

    public boolean buttonB = true;
    public boolean buttonA = true;
    public boolean buttonX = true;
    public boolean buttonY = true;
    public boolean buttonRight = true;
    public boolean buttonLeft = true;
    public boolean backButton = true;
    public boolean buttonRight2 = true;
    public boolean buttonLeft2 = true;
    public boolean buttonB2 = true;
    public boolean buttonA2 = true;
    public boolean buttonX2 = true;
    public boolean buttonY2 = true;
    public boolean dpadleft = true;

    public boolean slideToggle = true;

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



        //0.95 for grabbing from intake

        while(!isStarted() && !isStopRequested()){

            //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            shooterAngle.setPosition(0.35);
            shooter.setPosition(.57);

            shoulder.setPosition(.58);
            wrist.setPosition(.34);
            //finger.setPower(0);//positive = out
            //intakeServo.setPosition(0.78);
        }

        while (opModeIsActive()) {

            robot.refresh(robot.odometers);

            robot.mecanumDrive(gamepad1.right_stick_y, -gamepad1.right_stick_x, gamepad1.left_stick_x, speed);

            intakeServo.setPosition(intakePos2);

            if (gamepad1.a && buttonA || stickDropping){
                //intake
                intakePos = true;
                outputPos = false;
                shoulder.setPosition(.57);
                wrist.setPosition(.34);
                buttonA = false;
                slideEncoder = 0;

            }
            else if (gamepad1.b && buttonB){
                //placement
                intakePos = false;
                outputPos = true;
                shoulder.setPosition(.069);
                wrist.setPosition(.8);
                buttonB = false;
                slideEncoder = 400;

            }

            if (!gamepad1.a && !buttonA){
                buttonA = true;
            }
            else if (!gamepad1.b && !buttonB){
                buttonB = true;
            }


            //speed
            if (gamepad1.back && backButton){
                if (speed == 0.6){
                    speed = 1;
                }
                else{
                    speed = 0.6;
                }
                backButton = false;
            }
            else if (!gamepad1.back && !backButton){
                backButton  = true;
            }


            //slides fine adjust
            if (gamepad1.x && buttonX){
                slideEncoder -= 400;
                buttonX = false;
            }
            else if (gamepad1.y && buttonY){
                slideEncoder += 400;
                buttonY = false;
            }

            else if (!gamepad1.x && !buttonX){
                buttonX  = true;
            }
            else if (!gamepad1.y && !buttonY){
                buttonY  = true;
            }



            //intake
            if(gamepad1.right_trigger > 0.5){
                intake.setPower(0.85);
                finger.setPower(1);
            }
            else if(gamepad1.left_trigger > 0.5){
                if(intakePos){
                    intake.setPower(-0.5);
                    finger.setPower(.5);
                } else if(outputPos && !leftTrigger) {
                    finger.setPower(-1);
                    oldOutTime = outputTime.milliseconds();
                    leftTrigger = true;
                }
            }
            else {
                intake.setPower(0);
                //finger.setPower(0);
            }
            if(outputPos && leftTrigger && (oldOutTime + 250 < outputTime.milliseconds())){
                finger.setPower(0);
                leftTrigger = false;
            }
            else if(gamepad1.right_trigger < 0.5 && !leftTrigger){
                intake.setPower(0);
                finger.setPower(0);
            }




            //slides
            if(gamepad1.dpad_down){
                //slides retracted
                slidesRasied = false;
                slideEncoder = 800;
            }
            if(gamepad1.dpad_up){
                slidesRasied = true;
                slideEncoder = 2800;
            }
            if(gamepad1.dpad_right){
                slidesRasied = true;
                slideEncoder = 2000;
            }
            if(gamepad1.dpad_left){
                slidesRasied = true;
                slideEncoder = 1400;
            }

            //-------------------------------------------------------------------------------------------------------------------------------------//

            //Gamepad 2 controls

            //shooter
            if(gamepad2.right_bumper){
                shooterAngle.setPosition(0.21);
            }
            if(gamepad2.right_bumper && gamepad2.left_bumper){
                shooter.setPosition(0.5);
            }

            //Hang prep
            if (gamepad2.x && buttonX2){
                wrist.setPosition(.34);
                shoulder.setPosition(.125);
                //intakePos2 = .78;
            }

            //slide hanging
            else if (gamepad2.y && buttonY2){
                slideEncoder = 2800;
                buttonY2 = false;
            }
            else if (gamepad2.a && buttonA2){
                slideEncoder = 1000;
                buttonA2 = false;
            }
            else if (!gamepad2.a && !buttonA2){
                buttonA2  = true;
            }
            else if (!gamepad2.x && !buttonX2){
                buttonX2  = true;
            }
            else if (!gamepad2.y && !buttonY2){
                buttonY2  = true;
            }

            /*
            //shooter fine adjust
            if (gamepad2.dpad_right && buttonRight2){
                shooterAngle.setPosition(shooterAngle.getPosition() + .01);
                buttonRight2 = false;
            }
            else if (gamepad2.dpad_left && buttonLeft2){
                shooterAngle.setPosition(shooterAngle.getPosition() - .01);
                buttonLeft2 = false;
            }
            else if (!gamepad2.dpad_right && !buttonRight2){
                buttonRight2 = true;
            }
            else if (!gamepad2.dpad_left && !buttonLeft2){
                buttonLeft2 = true;
            }

             */

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
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            telemetry.addData("x",robot.GlobalX);
            telemetry.addData("y",robot.GlobalY);
            telemetry.addData("heading",Math.toDegrees(robot.GlobalHeading));
            telemetry.addData("",null);
            telemetry.addData("slide Encoder value", slideEncoder);
            telemetry.addData("shoulder", shoulder.getPosition());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("shooterAngle", shooterAngle.getPosition());
            telemetry.addData("speed", speed);
            telemetry.update();
        }
    }
    public void stick_up(){}
    public void stick_mid(){}
    public void stick_bottom(){}
    public void stick_hopper(){}
}