package org.firstinspires.ftc.teamcode.practiceRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotHardware;

@TeleOp(name="practiceRobotTeleop")
//@Disabled

public class practiceRobotTeleop extends LinearOpMode{

    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;

    public DcMotor slidesL = null;
    public DcMotor slidesR = null;
    public Servo clawL = null;
    public Servo clawR = null;
    public Servo wrist = null;


    public int slideEncoder = 0;

    public void runOpMode() throws InterruptedException {

        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist = hardwareMap.servo.get("wrist");
        clawR = hardwareMap.servo.get("clawR");
        clawL = hardwareMap.servo.get("clawL");

        slidesL = hardwareMap.dcMotor.get("slidesL");
        slidesR = hardwareMap.dcMotor.get("slidesR");

        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesR.setDirection(DcMotorSimple.Direction.REVERSE);


        clawL.setPosition(.6);
        clawR.setPosition(.5);
        //clawL .35 clawR .65 to collect
        wrist.setPosition(.4);
        waitForStart();

        while (opModeIsActive()) {

            motorRF.setPower((((-gamepad1.right_stick_y - gamepad1.right_stick_x) * 1) - (gamepad1.left_stick_x * 1)) * 1);
            motorRB.setPower((((-gamepad1.right_stick_y + gamepad1.right_stick_x) * 1) - (gamepad1.left_stick_x * 1)) * 1);
            motorLB.setPower((((-gamepad1.right_stick_y - gamepad1.right_stick_x) * 1) + (gamepad1.left_stick_x * 1)) * 1);
            motorLF.setPower((((-gamepad1.right_stick_y + gamepad1.right_stick_x) * 1) + (gamepad1.left_stick_x * 1)) * 1);


            if (gamepad1.left_bumper){
                //open
                clawL.setPosition(.6);
                clawR.setPosition(.5);
            }
            if (gamepad1.right_bumper){
                //close
                clawL.setPosition(.35);
                clawR.setPosition(.65);
            }

            if (gamepad1.dpad_up){
                slideEncoder = 1000;
            }
            if (gamepad1.dpad_up){
                slideEncoder = 800;
            }
            if (gamepad1.dpad_right){
                slideEncoder = 400;
            }
            if (gamepad1.dpad_down){
                slideEncoder = 0;
            }











            //slide movement code
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

        }
    }
}