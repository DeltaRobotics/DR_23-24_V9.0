package org.firstinspires.ftc.teamcode.gen2;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="blueWingSide")
//@Disabled

public class blueWingSide extends LinearOpMode{

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

    public Servo intakeServo = null;
    public CRServo finger = null;
    public Servo shoulderL = null;
    public Servo shoulderR = null;
    public Servo shooterAngle = null;
    public Servo wrist = null;
    public Servo pixScraper = null;

    RevBlinkinLedDriver blinkinLedDriver;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime servoTime = new ElapsedTime();

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

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
        pixScraper = hardwareMap.servo.get("pixScraper");

        slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);


        robot.resetDriveEncoders();
        intakeServo.setPosition(0.78);

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
            if(myPipeline.getRectMidpointX() >= 240 && myPipeline.getRectMidpointX() <= 370){
                telemetry.addData("location", 2);
                startingPos = 2;
            } else if(myPipeline.getRectMidpointX() >= 430 && myPipeline.getRectMidpointX() <= 600){
                telemetry.addData("location", 1);
                startingPos = 1;
            } else if(myPipeline.getRectMidpointX() >= 0 && myPipeline.getRectMidpointX() <= 160){
                telemetry.addData("location", 3);
                startingPos = 3;
            } else{
                telemetry.addData("location", "no head seen");
                startingPos = 1;
            }

            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);

            //telemetry.addData("x", myPipeline.getRectMidpointX());
            //telemetry.addData("y", myPipeline.getRectMidpointY());
            shooterAngle.setPosition(0);
            pixScraper.setPosition(.85);

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            telemetry.update();
        }
        camera.stopStreaming();
        if(startingPos == 1){
            camera.stopStreaming();
            robot.changeSpeed(.4,.4);
            //starting left
            //first drive forward

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(28,0,Math.toRadians(90),0);

            robot.goToPos(28,8,Math.toRadians(90),Math.toRadians(180));

            robot.goToPos(28,4,Math.toRadians(90),0);

            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            robot.goToPos(28,-10,Math.toRadians(90),0);

            robot.wait(500,robot.odometers);

            robot.goToPos(50,-10,Math.toRadians(-90),0);

            robot.wait(500,robot.odometers);

            shooterAngle.setPosition(0.33);
            pixScraper.setPosition(.7);

            robot.wait(5000,robot.odometers);

            robot.changeAccuracy(.25,Math.toRadians(1));

            robot.changeSpeed(.6,.6);

            //move past bar
            robot.goToPos(50,80,Math.toRadians(-90),0);

            robot.duelServoController(.57,shoulderL,shoulderR);

            robot.changeAccuracy(.25, Math.toRadians(1));

            robot.goToPos(23,80,Math.toRadians(-90),Math.toRadians(90));

            robot.changeAccuracy(1, Math.toRadians(1));

            robot.changeSpeed(.4,.4);

            robot.wait(500,robot.odometers);

            wrist.setPosition(.8);

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 3000) {

                slidesL.setTargetPosition(600);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(600);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.mecanumDrive(0,0,0,0.3);

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(1000, robot.odometers);

            finger.setPower(0);

            robot.wait(500, robot.odometers);

            robot.duelServoController(.04,shoulderL,shoulderR);
            wrist.setPosition(.34);

            ElapsedTime driveG = new ElapsedTime();

            //drop slides
            while(slidesR.getCurrentPosition() > 20) {

                slidesL.setTargetPosition(0);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(0);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.refresh(robot.odometers);

            }

            robot.wait(2000, robot.odometers);

        }
        else if(startingPos == 2){
            camera.stopStreaming();
            robot.changeSpeed(.4,.4);
            //starting middle
            //first drive forward

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(47,0,0,0);

            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            robot.goToPos(50,-10,Math.toRadians(-90),0);

            robot.wait(500,robot.odometers);

            shooterAngle.setPosition(0.33);
            pixScraper.setPosition(.75);

            robot.wait(10000,robot.odometers);

            robot.changeSpeed(.6,.6);

            robot.wait(500,robot.odometers);

            robot.goToPos(50,80,Math.toRadians(-90),0);

            robot.duelServoController(.57,shoulderL,shoulderR);

            robot.changeAccuracy(.25, Math.toRadians(1));

            //ready to place
            robot.goToPos(27,80,Math.toRadians(-90),Math.toRadians(90));

            robot.changeAccuracy(1, Math.toRadians(1));

            robot.changeSpeed(.4,.4);

            wrist.setPosition(.8);

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 3000) {

                slidesL.setTargetPosition(300);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(300);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(1000, robot.odometers);

            finger.setPower(0);

            robot.wait(500, robot.odometers);

            robot.duelServoController(.04,shoulderL,shoulderR);
            wrist.setPosition(.34);

            ElapsedTime driveG = new ElapsedTime();

            //drop slides
            while(slidesR.getCurrentPosition() > 20) {

                slidesL.setTargetPosition(0);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(0);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.refresh(robot.odometers);

            }

            robot.wait(2000, robot.odometers);

        }
        else if(startingPos == 3){
            camera.stopStreaming();
            robot.changeSpeed(.4,.4);
            //starting right

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(42,-10,0,0);

            //purple pixel
            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            robot.goToPos(47,-10,0,0);

            robot.wait(1000,robot.odometers);

            //ready for long move
            robot.goToPos(50,-10,Math.toRadians(-90),0);

            robot.wait(10000,robot.odometers);

            shooterAngle.setPosition(0.33);
            pixScraper.setPosition(.75);

            robot.wait(500,robot.odometers);

            robot.changeSpeed(.6,.6);

            //spin
            robot.goToPos(50,80,Math.toRadians(-90),0);

            robot.duelServoController(.57,shoulderL,shoulderR);

            robot.changeAccuracy(.25, Math.toRadians(1));

            //move to bord
            robot.goToPos(34,80,Math.toRadians(-90),Math.toRadians(90));

            robot.changeAccuracy(1, Math.toRadians(1));

            robot.changeSpeed(.4,.4);

            wrist.setPosition(.8);

            robot.wait(500,robot.odometers);

            ElapsedTime driveF = new ElapsedTime();

            //move slides up
            while(driveF.milliseconds() < 3000) {

                slidesL.setTargetPosition(600);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(600);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(1000, robot.odometers);

            finger.setPower(0);

            robot.wait(500, robot.odometers);

            //move shoulder down
            robot.duelServoController(.04,shoulderL,shoulderR);
            wrist.setPosition(.34);

            //drop slides
            while(slidesR.getCurrentPosition() > 20) {

                slidesL.setTargetPosition(0);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(0);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.refresh(robot.odometers);

            }

            robot.wait(2000,robot.odometers);

            ElapsedTime driveG = new ElapsedTime();

            //drop slides
            while(driveG.milliseconds() < 250) {

                slidesL.setTargetPosition(-50);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(-50);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.refresh(robot.odometers);

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