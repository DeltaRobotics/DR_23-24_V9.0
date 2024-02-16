package org.firstinspires.ftc.teamcode.gen2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHardware;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name="ghettoAuto")
//@Disabled
@Config
public class ghettoAuto extends LinearOpMode{

    private OpenCvCamera camera;//find webcam statement

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Yellow Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 105.0, 150.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 200.0, 255.0);

    double poleOffset = 0;
    double poleOffsetPower = 0;

    public static int startingPos = 2;

    double doubleLoop = 10;
    boolean pixelGrab = false;

    int slideEncoder = 400;

    int x;
    int y;
    double finalAngle;
    double followAngle;

    //non-wheels
    public DcMotor slidesL = null;
    public DcMotor slidesR = null;
    public DcMotor intake = null;

    public Servo intakeServo = null;
    public CRServo finger = null;
    public Servo shoulderL = null;
    public Servo shoulderR = null;
    public Servo shooterAngle = null;
    public Servo wrist = null;

    int finding = 0;

    public static double turnAngle = 90;


    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime servoTime = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        robotHardware robot = new robotHardware(hardwareMap);

        intake = hardwareMap.dcMotor.get("intake");

        intakeServo = hardwareMap.servo.get("intakeServo");
        shoulderL = hardwareMap.servo.get("shoulderL");
        shoulderR = hardwareMap.servo.get("shoulderR");

        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");
        shooterAngle = hardwareMap.servo.get("shooterAngle");
        finger = hardwareMap.crservo.get("finger");
        wrist = hardwareMap.servo.get("wrist");

        slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);


        robot.resetDriveEncoders();
        //intakeServo.setPosition(0.78);

        //FtcDashboard.getInstance().startCameraStream(camera, 10);

        //!isStarted() && !isStopRequested()
        //this replaces waitForStart()

        while(!isStarted() && !isStopRequested()){

            //startingPos = 1;
            //1 = stafe tuning. 2 = turn tuning. 3 = fords tuning
            shooterAngle.setPosition(0);

            wrist.setPosition(.34);

            robot.changeSpeed(1,1);
            robot.changeAccuracy(1,Math.toRadians(1));

            telemetry.addData("1",1);
            telemetry.addData("ghetto",0);
            telemetry.addData("finding", finding);
            telemetry.update();
        }
        //camera.stopStreaming();
        if(startingPos == 1){
            while(true) {
                x = 0;
                y = 60;
                finalAngle = Math.toRadians(0);
                followAngle = Math.toRadians(90);

                while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
                    telemetry.addData("ghetto", Math.abs(robot.goToPosSingle(x, y, finalAngle, followAngle)));
                    telemetry.addData("1", 1);
                    telemetry.update();
                }

                robot.mecanumDrive(0, 0, 0, 0);
                robot.wait(1000, robot.odometers);

                x = 0;
                y = 0;
                followAngle = Math.toRadians(-90);
                finalAngle = Math.toRadians(0);

                while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
                    telemetry.addData("ghetto", Math.abs(robot.goToPosSingle(x, y, finalAngle, followAngle)));
                    telemetry.addData("1", 1);
                    telemetry.update();
                }

                robot.mecanumDrive(0, 0, 0, 0);
                robot.wait(1000, robot.odometers);
            }
        }
        else if(startingPos == 2){

            while(true) {
                x = 10;
                y = 0;
                finalAngle = Math.toRadians(turnAngle);
                followAngle = 0;

                while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
                    telemetry.addData("ghetto", Math.abs(robot.goToPosSingle(x, y, finalAngle, followAngle)));
                    telemetry.addData("1", 1);
                    telemetry.update();
                }

                robot.mecanumDrive(0,0,0,0);
                robot.wait(1000, robot.odometers);

                x = 10;
                y = 0;
                finalAngle = Math.toRadians(0);

                while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
                    telemetry.addData("ghetto", Math.abs(robot.goToPosSingle(x, y, finalAngle, followAngle)));
                    telemetry.addData("1", 1);
                    telemetry.update();
                }

                robot.mecanumDrive(0,0,0,0);
                robot.wait(1000, robot.odometers);
            }

        }
        else if(startingPos == 3){
            while(true) {
                x = 60;
                y = 0;
                finalAngle = Math.toRadians(0);
                followAngle = Math.toRadians(0);

                while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
                    telemetry.addData("ghetto", Math.abs(robot.goToPosSingle(x, y, finalAngle, followAngle)));
                    telemetry.addData("1", 1);
                    telemetry.update();
                }

                robot.mecanumDrive(0, 0, 0, 0);
                robot.wait(1000, robot.odometers);

                x = 0;
                y = 0;
                followAngle = Math.toRadians(180);
                finalAngle = Math.toRadians(0);

                while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
                    telemetry.addData("ghetto", Math.abs(robot.goToPosSingle(x, y, finalAngle, followAngle)));
                    telemetry.addData("1", 1);
                    telemetry.update();
                }

                robot.mecanumDrive(0, 0, 0, 0);
                robot.wait(1000, robot.odometers);
            }
        }
    }

    public void raiseSlides(int destination){
        if(slidesR.getCurrentPosition() < 20 && destination < 20){
            slidesL.setTargetPosition(destination);
            slidesL.setPower(0);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slidesR.setTargetPosition(destination);
            slidesR.setPower(0);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            slidesL.setTargetPosition(destination);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slidesR.setTargetPosition(destination);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

}