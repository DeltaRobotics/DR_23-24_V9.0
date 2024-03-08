package org.firstinspires.ftc.teamcode.gen2;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PoleDetectionPipeline;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="igneaBlueBoardSideV2Thor")
//@Disabled

public class igneaBlueBoardSideV2Thor extends LinearOpMode{

    private OpenCvCamera camera;//find webcam statement

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Yellow Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 105.0, 150.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 200.0, 255.0);

    double poleOffset = 0;
    double poleOffsetPower = 0;

    int startingPos;

    double doubleLoop = 10;
    boolean pixelGrab = false;

    int slideEncoder = 400;

    int x;
    int y;
    double finalAngle;

    //non-wheels
    public DcMotor slidesL = null;
    public DcMotor slidesR = null;
    public DcMotor intake = null;

    public CRServo finger = null;
    public Servo wrist = null;
    public Servo shoulderL = null;
    public Servo shoulderR = null;
    public Servo intakeServo = null;
    public Servo shooterAngle = null;

    RevBlinkinLedDriver blinkinLedDriver;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime servoTime = new ElapsedTime();

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        robotHardware robot = new robotHardware(hardwareMap);

        intake = hardwareMap.dcMotor.get("intake");

        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");

        finger = hardwareMap.crservo.get("finger");
        wrist = hardwareMap.servo.get("wrist");
        shoulderL = hardwareMap.servo.get("shoulderL");
        shoulderR = hardwareMap.servo.get("shoulderR");
        intakeServo = hardwareMap.servo.get("intakeServo");
        shooterAngle = hardwareMap.servo.get("shooterAngle");

        slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.resetDriveEncoders();

        // OpenCV lift webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV PoleDetectionPipeline
        PoleDetectionPipeline myPipeline;
        camera.setPipeline(myPipeline = new PoleDetectionPipeline());
        // Configuration of PoleDetectionPipeline
        myPipeline.ConfigurePipeline(0, 0,50,50,  CAMERA_WIDTH, CAMERA_HEIGHT);
        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        telemetry.addData("Pipeline Ready", 0);

        telemetry.update();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("camera not started", myPipeline.debug);
            }
        });
        //FtcDashboard.getInstance().startCameraStream(camera, 10);

        //!isStarted() && !isStopRequested()
        //this replaces waitForStart()

        while(!isStarted() && !isStopRequested()){
            if(myPipeline.getRectMidpointX() >= 170 && myPipeline.getRectMidpointX() <= 370){
                telemetry.addData("location", 2);
                startingPos = 2;
            } else if(myPipeline.getRectMidpointX() >= 380 && myPipeline.getRectMidpointX() <= 625){
                telemetry.addData("location", 1);
                startingPos = 1;
            } else if(myPipeline.getRectMidpointX() >= 0 && myPipeline.getRectMidpointX() <= 110){
                telemetry.addData("location", 3);
                startingPos = 3;
            } else{
                telemetry.addData("location", "no head seen");
                startingPos = 3;
            }


            intakeServo.setPosition(0.78);
            shooterAngle.setPosition(0);

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);

            telemetry.update();
        }
        camera.stopStreaming();

        //TODO adjust speeds in 2 and 3 like in 1
        if(startingPos == 1){
            camera.stopStreaming();
            //starting left
            //first drive forward
            robot.changeSpeed(1,1);

            while(slidesR.getCurrentPosition() < 380) {
                raiseSlides(400);
            }

            robot.wait(15000, robot.odometers);

            robot.duelServoController(.57,shoulderL,shoulderR);

            wrist.setPosition(.86);

            robot.changeAccuracy(.25,Math.toRadians(1));

            //move to place
            robot.goToPos(19, 34, Math.toRadians(-90), 0);

            robot.changeAccuracy(1,Math.toRadians(1));

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 750) {

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(1000, robot.odometers);

            finger.setPower(0);

            robot.changeSpeed(.4,.4);

            robot.duelServoController(.2,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(19, 34, Math.toRadians(-90), Math.toRadians(180));

            robot.changeSpeed(1,1);

            robot.goToPos(5, 5, Math.toRadians(180), Math.toRadians(90));

            robot.changeSpeed(.8,.8);

            robot.wait(500, robot.odometers);

            //move to place purple pixel
            robot.goToPos(19,8,Math.toRadians(180), Math.toRadians(180));

            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);


            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            //drop off purple pixel
            robot.goToPos(5,8,Math.toRadians(180), 0);

            //ready to time drive
            robot.goToPos(5,30,Math.toRadians(-90), 0);

            robot.mecanumDrive(0,0,0,0);
            while(slidesR.getCurrentPosition() > 20) {
                raiseSlides(0);
            }

            robot.wait(500, robot.odometers);
        }
        else if(startingPos == 2){
            camera.stopStreaming();
            //starting middle
            //first drive forward
            robot.changeSpeed(1,1);
            robot.changeAccuracy(.25,Math.toRadians(1));

            while(slidesR.getCurrentPosition() < 380) {
                raiseSlides(400);
            }

            robot.wait(13000, robot.odometers);

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.duelServoController(.57,shoulderL,shoulderR);

            wrist.setPosition(.86);

            //move to place
            robot.goToPos(25.5, 34, Math.toRadians(-90), 0);

            robot.changeAccuracy(1,Math.toRadians(1));

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 750) {

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(1500, robot.odometers);

            finger.setPower(0);

            robot.changeSpeed(.4,.4);

            robot.duelServoController(.2,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(19, 30, Math.toRadians(-90), Math.toRadians(180));

            robot.changeSpeed(1,1);

            robot.goToPos(5, 5, Math.toRadians(180), Math.toRadians(90));

            robot.changeSpeed(.8,.8);

            robot.wait(500, robot.odometers);

            //move to place purple pixel
            robot.goToPos(28,2,Math.toRadians(180),Math.toRadians(180));

            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            //drop off purple pixel
            robot.goToPos(10,5,Math.toRadians(180), 0);

            //ready to time drive
            robot.goToPos(5,30,Math.toRadians(-90), 0);


            robot.mecanumDrive(0,0,0,0);
            while(slidesR.getCurrentPosition() > 20) {
                raiseSlides(0);
            }

            robot.wait(500, robot.odometers);
        }
        else if(startingPos == 3){
            camera.stopStreaming();
            //starting right
            //place purple pixel

            robot.changeSpeed(1,1);

            robot.changeAccuracy(.25,Math.toRadians(1));

            while(slidesR.getCurrentPosition() < 380) {
                raiseSlides(400);
            }

            robot.wait(13000, robot.odometers);

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.duelServoController(.57,shoulderL,shoulderR);

            wrist.setPosition(.86);

            //move to place
            robot.goToPos(32, 34, Math.toRadians(-90), 0);

            robot.changeAccuracy(1,Math.toRadians(1));

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 1250) {

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(1000, robot.odometers);

            finger.setPower(0);

            robot.changeSpeed(.4,.4);

            robot.duelServoController(.2,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(34, 34, Math.toRadians(-90), Math.toRadians(180));

            robot.changeSpeed(1,1);

            robot.goToPos(5, 5, Math.toRadians(180), 0);

            robot.wait(500, robot.odometers);

            robot.changeSpeed(.8,.8);

            //move to place purple pixel
            robot.goToPos(28,0,Math.toRadians(-90),Math.toRadians(180));

            robot.goToPos(28,-9,Math.toRadians(-90),Math.toRadians(180));

            robot.goToPos(28,-5,Math.toRadians(-90),Math.toRadians(90));

            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            //drop off purple pixel
            robot.goToPos(28,5,Math.toRadians(-90), 0);

            //ready to time drive
            robot.goToPos(5,5,Math.toRadians(-90), Math.toRadians(90));

            robot.goToPos(5,30,Math.toRadians(-90), 0);

            robot.mecanumDrive(0,0,0,0);
            while(slidesR.getCurrentPosition() > 20) {
                raiseSlides(0);
            }

            robot.wait(500, robot.odometers);
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