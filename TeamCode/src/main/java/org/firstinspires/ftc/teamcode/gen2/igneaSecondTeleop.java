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

    public boolean unoPixo = false;

    public boolean rightBumper = true;
    public boolean leftBumper = true;

    public double speed = .75;

    public double wristPos = .79;

    public boolean buttonB = true;
    public boolean buttonA = true;
    public boolean buttonX = true;
    public boolean buttonY = true;

    RevBlinkinLedDriver blinkinLedDriver;

    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        ElapsedTime servoTime = new ElapsedTime();

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

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        shooter.setPosition(0.75);


        //0.95 for grabbing from intake

        while(!isStarted() && !isStopRequested()){

            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            shooterAngle.setPosition(0.5);
            shooter.setPosition(.54);

            shoulder.setPosition(.4);
            wrist.setPosition(.4);
            finger.setPower(0);//positive = out
            intakeServo.setPosition(0.78);
        }

        while (opModeIsActive()) {

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed);

            intakeServo.setPosition(.56);

            //shooterAngle adjust
            if (gamepad1.a && buttonA || stickDropping){
                //intake
                shoulder.setPosition(.59);
                wrist.setPosition(.4);
                buttonA = false;
            }
            else if (gamepad1.b && buttonB){
                //placement
                shoulder.setPosition(.1);
                wrist.setPosition(.9);
                buttonB = false;
            }

            else if (gamepad1.x && buttonX){
                shoulder.setPosition(shoulder.getPosition() + .005);
                buttonX = false;
            }
            else if (gamepad1.y && buttonY){
                shoulder.setPosition(shoulder.getPosition() - .005);
                buttonY = false;
            }


            if (!gamepad1.a && !buttonA){
                buttonA = true;
            }
            else if (!gamepad1.b && !buttonB){
                buttonB = true;
            }

            else if (!gamepad1.x && !buttonX){
                buttonX  = true;
            }
            else if (!gamepad1.y && !buttonY){
                buttonY  = true;
            }


            telemetry.addData("shoulder", shoulder.getPosition());
            telemetry.addData("wrist", wrist.getPosition());

            if(gamepad1.right_trigger > 0.5){
                intake.setPower(0.85);
                finger.setPower(-1);
            }
            else if(gamepad1.left_trigger > 0.5){
                intake.setPower(-0.5);
                finger.setPower(1);
            }
            else {
                intake.setPower(0);
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

            //shooter
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