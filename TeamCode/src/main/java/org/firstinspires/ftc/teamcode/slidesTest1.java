package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="slidesTest1")
@Disabled

public class slidesTest1 extends LinearOpMode{

    int slideEncoder = 0;
    double slidePower = .5;

    //non-wheels
    public Servo launcher = null;
    public DcMotor slidesL = null;
    public DcMotor slidesR = null;
    public Servo clawL = null;
    public Servo clawR = null;
    public Servo wrist = null;

    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        launcher = hardwareMap.servo.get("launcher");
        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        wrist = hardwareMap.servo.get("wrist");

        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.resetDriveEncoders();

        launcher.setPosition(0.85);

        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 0.75);

            if(gamepad1.right_bumper && gamepad1.left_bumper){
                //launch drone
                launcher.setPosition(0.65);
            }
            if(gamepad1.a){
                //reload
                launcher.setPosition(0.85);
            }

            //slides max is around 3500 encoder tics

            if(gamepad1.a){
                //slides down
                slidePower = .5;
                slideEncoder = 25;
            }
            else if(gamepad1.b){
                //slides max
                slidePower = .7;
                slideEncoder = 3500;
            }

            slidesR.setTargetPosition(slideEncoder);
            slidesR.setPower(slidePower);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slidesL.setTargetPosition(slideEncoder);
            slidesL.setPower(slidePower);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            telemetry.addData("slide encoder",slideEncoder);
            telemetry.addData("servo", launcher.getPosition());
            telemetry.update();
        }
    }
}