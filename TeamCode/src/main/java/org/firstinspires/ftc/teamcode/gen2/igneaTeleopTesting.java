package org.firstinspires.ftc.teamcode.gen2;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHardware;

@TeleOp(name="igneaTeleopTesting")
//@Disabled

public class igneaTeleopTesting extends LinearOpMode{

    public CRServo finger = null;
    public Servo wrist = null;
    public Servo shoulder = null;
    public Servo intakeServo = null;
    public Servo shooter = null;
    public Servo shooterAngle = null;

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

    public boolean buttonPress = true;


    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        ElapsedTime servoTime = new ElapsedTime();

        finger = hardwareMap.crservo.get("finger");
        wrist = hardwareMap.servo.get("wrist");
        shoulder = hardwareMap.servo.get("shoulder");
        intakeServo = hardwareMap.servo.get("intakeServo");
        shooter = hardwareMap.servo.get("shooter");
        shooterAngle = hardwareMap.servo.get("shooterAngle");

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


            shooterAngle.setPosition(0.35);
            shooter.setPosition(.57);

            shoulder.setPosition(.58);
            wrist.setPosition(.34);

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

            if (gamepad1.a && buttonPress){
                shoulder.setPosition(shoulder.getPosition()+0.01);
                buttonPress = false;
            }
            else if (gamepad1.b && buttonPress){
                shoulder.setPosition(shoulder.getPosition()-0.01);
                buttonPress = false;
            }

            if (!gamepad1.a && !gamepad1.b && !buttonPress){
                buttonPress = true;
            }









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