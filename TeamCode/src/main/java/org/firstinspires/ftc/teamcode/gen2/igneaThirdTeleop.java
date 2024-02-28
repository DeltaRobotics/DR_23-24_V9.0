package org.firstinspires.ftc.teamcode.gen2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHardware;

@TeleOp(name="igneaThirdTeleop")
//@Disabled

public class igneaThirdTeleop extends LinearOpMode{

    public CRServo finger = null;
    public Servo shooterAngle = null;
    public Servo wrist = null;
    public Servo shoulderL = null;
    public Servo shoulderR = null;
    public Servo intakeServo = null;
    public Servo shooter = null;
    public Servo pixMover = null;
    public Servo pixScraper = null;

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

    RevBlinkinLedDriver blinkinLedDriver;


    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        robot.resetDriveEncoders();

        ElapsedTime outputTime = new ElapsedTime();

        finger = hardwareMap.crservo.get("finger");
        shooterAngle = hardwareMap.servo.get("shooterAngle");
        wrist = hardwareMap.servo.get("wrist");
        shoulderL = hardwareMap.servo.get("shoulderL");
        shoulderR = hardwareMap.servo.get("shoulderR");
        intakeServo = hardwareMap.servo.get("intakeServo");
        shooter = hardwareMap.servo.get("shooter");
        pixMover = hardwareMap.servo.get("pixMover");
        pixScraper = hardwareMap.servo.get("pixScraper");

        intake = hardwareMap.dcMotor.get("intake");
        slidesL = hardwareMap.dcMotor.get("slidesL");
        slidesR = hardwareMap.dcMotor.get("slidesR");

        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);



        //0.95 for grabbing from intake

        while(!isStarted() && !isStopRequested()){

            //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            shooterAngle.setPosition(0.30);
            shooter.setPosition(.57);

            robot.duelServoController(.05,shoulderL,shoulderR);
            //shoulderL.setPosition(0);
            //shoulderR.setPosition(1);

            wrist.setPosition(.34);
            //finger.setPower(0);//positive = out
            //intakeServo.setPosition(0.78);

            pixScraper.setPosition(.95);
            pixMover.setPosition(.4);

            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
        }

        while (opModeIsActive()) {

            robot.refresh(robot.odometers);

            robot.mecanumDrive(gamepad1.right_stick_y, -gamepad1.right_stick_x, gamepad1.left_stick_x, speed);

            intakeServo.setPosition(intakePos2);

            if (gamepad1.left_bumper && buttonA || stickDropping){
                //intake
                intakePos = true;
                outputPos = false;
                wrist.setPosition(.34);

                robot.duelServoController(.05,shoulderL,shoulderR);

                buttonA = false;
                slideEncoder = 0;

            }
            else if (gamepad1.right_bumper && buttonB){
                //placement
                intakePos = false;
                outputPos = true;
                robot.duelServoController(.57,shoulderL,shoulderR);
                wrist.setPosition(.86);
                buttonB = false;
                slideEncoder = 400;
                speed = .4;

            }

            if (!gamepad1.left_bumper && !buttonA){
                buttonA = true;
            }
            else if (!gamepad1.right_bumper && !buttonB){
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
                slideEncoder -= 200;
                buttonX = false;
            }
            else if (gamepad1.y && buttonY){
                slideEncoder += 200;
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
                    finger.setPower(-1);
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
                shooterAngle.setPosition(0.2);
            }
            if(gamepad2.right_bumper && gamepad2.left_bumper){
                shooter.setPosition(0.5);
            }

            //Hang prep
            if (gamepad2.x && buttonX2){
                wrist.setPosition(.34);
                robot.duelServoController(.46,shoulderL,shoulderR);
                //intakePos2 = .78;
            }

            //slide hanging
            else if (gamepad2.y && buttonY2){
                slideEncoder = 3500;
                buttonY2 = false;
            }
            else if (gamepad2.a && buttonA2){
                slideEncoder = 1000;
                robot.duelServoController(.8,shoulderL,shoulderR);
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

            //robot.servoFineAdjust(pixScraper, gamepad2.dpad_up, gamepad2.dpad_down, .01);

            //pixMover
            if (gamepad2.dpad_up && buttonRight2){
                //in robot
                pixMover.setPosition(.4);
                buttonRight2 = false;
            }
            else if (gamepad2.dpad_down && buttonLeft2){
                //moving pix
                pixMover.setPosition(.57);
                buttonLeft2 = false;
            }
            else if (!gamepad2.dpad_up && !buttonRight2){
                buttonRight2 = true;
            }
            else if (!gamepad2.dpad_down && !buttonLeft2){
                buttonLeft2 = true;
            }


            //pixScraper
            if (gamepad2.right_trigger > .3){
                pixScraper.setPosition(pixScraper.getPosition()-.005);
            }
            else if (gamepad2.left_trigger > .3){
                pixScraper.setPosition(.9);
            }


            //shoulder fine adjust
            /*
            if (gamepad2.dpad_right && buttonRight2){
                robot.duelServoController(shoulderR.getPosition()+.01,shoulderL,shoulderR);
                buttonRight2 = false;
            }
            else if (gamepad2.dpad_left && buttonLeft2){
                robot.duelServoController(shoulderR.getPosition()-.01,shoulderL,shoulderR);
                buttonLeft2 = false;
            }
            else if (!gamepad2.dpad_right && !buttonRight2){
                buttonRight2 = true;
            }
            else if (!gamepad2.dpad_left && !buttonLeft2){
                buttonLeft2 = true;
            }

             */


            if (outputPos){
                speed = .4;
            }
            else{
                speed = 1;
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
            telemetry.addData("shoulder", shoulderR.getPosition());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("shooterAngle", shooterAngle.getPosition());
            telemetry.addData("speed", speed);
            telemetry.addData("pixMover", pixMover.getPosition());
            telemetry.addData("pixScraper", pixScraper.getPosition());
            telemetry.update();
        }
    }
    public void stick_up(){}
    public void stick_mid(){}
    public void stick_bottom(){}
    public void stick_hopper(){}
}