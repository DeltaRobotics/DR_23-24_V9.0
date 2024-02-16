package org.firstinspires.ftc.teamcode.practiceRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    ElapsedTime timeOG = new ElapsedTime();
    public double time = -1500;
    ElapsedTime timeOG2 = new ElapsedTime();
    public double time2 = -1500;

    public boolean buttonLeft_Bumper = true;
    public boolean buttonRight_Bumper = true;

    public String testing = null;

    public void runOpMode() throws InterruptedException {

        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);

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

            //motorRF.setPower((((-gamepad1.right_stick_y - gamepad1.right_stick_x) * 1) - (gamepad1.left_stick_x * 1)) * 1);

            motorRF.setPower((((-gamepad1.left_stick_y - gamepad1.left_stick_x) * 1) - (gamepad1.right_stick_x * 1)) * 1);
            motorRB.setPower((((-gamepad1.left_stick_y + gamepad1.left_stick_x) * 1) - (gamepad1.right_stick_x * 1)) * 1);
            motorLB.setPower((((-gamepad1.left_stick_y - gamepad1.left_stick_x) * 1) + (gamepad1.right_stick_x * 1)) * 1);
            motorLF.setPower((((-gamepad1.left_stick_y + gamepad1.left_stick_x) * 1) + (gamepad1.right_stick_x * 1)) * 1);


            if ((gamepad1.left_bumper && buttonLeft_Bumper) || time2 + 750 > timeOG2.milliseconds()){
                //open
                if (time2 + 800 < timeOG2.milliseconds()){//setting timer
                    time2 = timeOG2.milliseconds();
                }

                if(slidesR.getCurrentPosition() > 500){
                    wrist.setPosition(.57);
                    if (time2 + 250 < timeOG2.milliseconds()){
                        clawL.setPosition(.65);
                        clawR.setPosition(.45);
                    }
                }
                else{
                    clawL.setPosition(.65);
                    clawR.setPosition(.45);
                }

                if (slidesR.getCurrentPosition() < 800) {
                    if (time2 + 250 < timeOG2.milliseconds()) {//waiting to move wrist up
                        slideEncoder = 300;
                    }
                    if (time2 + 500 < timeOG2.milliseconds()) {//waiting after wrist to move slides down
                        wrist.setPosition(.57);
                    }
                }
                testing = "open";
                buttonLeft_Bumper = false;
            }



            if ((gamepad1.right_bumper && buttonRight_Bumper) || time + 1000 > timeOG.milliseconds()){
                //close
                if (time + 1100 < timeOG.milliseconds()){//setting timer
                    time = timeOG.milliseconds();
                }

                clawL.setPosition(.4);//closing claw
                clawR.setPosition(.65);

                if (time + 500 < timeOG.milliseconds()) {//waiting to move wrist up
                    wrist.setPosition(.4);
                }
                if (time + 750 < timeOG.milliseconds()) {//waiting after wrist to move slides down
                    slideEncoder = 0;
                }

                testing = "close";

                buttonRight_Bumper = false;
            }



            if (!gamepad1.right_bumper && !buttonRight_Bumper){
                buttonRight_Bumper = true;
            }
            if (!gamepad1.left_bumper && !buttonLeft_Bumper){
                buttonLeft_Bumper = true;
            }

            if (gamepad1.dpad_up){
                slideEncoder = 3500;
            }
            if (gamepad1.dpad_left){
                slideEncoder = 2500;
            }
            if (gamepad1.dpad_right){
                slideEncoder = 1500;
            }
            if (gamepad1.dpad_down){
                slideEncoder = 0;
                wrist.setPosition(.4);
            }


            //slide movement code
            if(slidesR.getCurrentPosition() < 10 && slideEncoder < 20){
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
        }
    }
}