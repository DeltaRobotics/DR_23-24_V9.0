package org.firstinspires.ftc.teamcode.gen2;

//import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous(name="blueWingSideV2")
//@Disabled

public class blueWingSideV2 extends LinearOpMode{

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

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime servoTime = new ElapsedTime();

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

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
            } else if(myPipeline.getRectMidpointX() >= 430 && myPipeline.getRectMidpointX() <= 550){
                telemetry.addData("location", 1);
                startingPos = 1;
            } else if(myPipeline.getRectMidpointX() >= 0 && myPipeline.getRectMidpointX() <= 160){
                telemetry.addData("location", 3);
                startingPos = 3;
            } else{
                telemetry.addData("location", "no head seen");
                startingPos = 1;
            }

            //telemetry.addData("x", myPipeline.getRectMidpointX());
            //telemetry.addData("y", myPipeline.getRectMidpointY());
            shooterAngle.setPosition(0);

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);
            pixScraper.setPosition(.85);

            telemetry.update();
        }
        camera.stopStreaming();
        if(startingPos == 1){
            camera.stopStreaming();
            robot.changeSpeed(1,1);
            //starting left
            //first drive forward

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(28,0,Math.toRadians(90),0);

            robot.changeSpeed(.4,.4);

            robot.goToPos(28,8,Math.toRadians(90),Math.toRadians(180));

            robot.goToPos(28,4,Math.toRadians(90),0);

            robot.changeSpeed(1,1);

            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            robot.goToPos(28,-10,Math.toRadians(90),0);

            robot.changeSpeed(.6,.6);

            robot.changeAccuracy(.15,Math.toRadians(1));

            robot.goToPos(49.5,-16.5,Math.toRadians(-90),Math.toRadians(-90));

            robot.wait(5000,robot.odometers);

            robot.changeAccuracy(1,Math.toRadians(1));

            pixScraper.setPosition(.62);

            robot.wait(500,robot.odometers);

            robot.changeSpeed(1,1);

            robot.goToPos(49,-5,Math.toRadians(-90),0);

            robot.duelServoController(.05,shoulderL,shoulderR);

            pixScraper.setPosition(.75);

            robot.changeSpeed(.4,.4);

            intake.setPower(.85);
            finger.setPower(1);

            robot.changeAccuracy(.25,Math.toRadians(1));

            //collecting pixel
            robot.goToPos(50,-18.25,Math.toRadians(-90),Math.toRadians(180));

            robot.wait(1000,robot.odometers);

            robot.changeAccuracy(1,Math.toRadians(1));

            robot.goToPos(49,-15,Math.toRadians(-90),0);

            shooterAngle.setPosition(0.33);
            pixScraper.setPosition(.7);

            robot.changeSpeed(1,1);

            //move past bar
            robot.goToPos(50,80,Math.toRadians(-90),0);

            finger.setPower(0);
            intake.setPower(0);

            robot.duelServoController(.57,shoulderL,shoulderR);

            wrist.setPosition(.8);

            robot.changeAccuracy(1, Math.toRadians(1));

            robot.changeSpeed(0.7,0.7);

            robot.goToPos(34,88,Math.toRadians(-90),Math.toRadians(90));

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 500) {

                slidesL.setTargetPosition(100);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(100);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.mecanumDrive(0,0,0,0.3);

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(250, robot.odometers);

            finger.setPower(0);

            robot.wait(500, robot.odometers);

            robot.changeAccuracy(.25, Math.toRadians(1));
            robot.goToPos(23,88,Math.toRadians(-90),Math.toRadians(90));

            ElapsedTime driveH = new ElapsedTime();

            //drive to the backdrop
            while(driveH.milliseconds() < 500) {

                slidesL.setTargetPosition(400);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(400);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.mecanumDrive(0,0,0,0.3);

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(750, robot.odometers);

            finger.setPower(0);

            robot.wait(500, robot.odometers);

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.wait(500, robot.odometers);

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


        }
        else if(startingPos == 2){
            camera.stopStreaming();
            robot.changeSpeed(1,1);
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

            robot.goToPos(50,-10,0,0);

            robot.wait(5000,robot.odometers);

            robot.changeSpeed(.6,.6);

            robot.changeAccuracy(.15,Math.toRadians(1));

            robot.goToPos(49.5,-16.5,Math.toRadians(-90),Math.toRadians(180));

            robot.wait(5000,robot.odometers);

            robot.changeAccuracy(1,Math.toRadians(1));

            pixScraper.setPosition(.62);

            robot.wait(500,robot.odometers);

            robot.changeSpeed(1,1);

            robot.goToPos(49,-5,Math.toRadians(-90),0);

            robot.duelServoController(.05,shoulderL,shoulderR);

            pixScraper.setPosition(.75);

            robot.changeSpeed(.4,.4);

            intake.setPower(.8);
            finger.setPower(1);

            robot.changeAccuracy(.25,Math.toRadians(1));

            //collecting pixel
            robot.goToPos(50,-18.25,Math.toRadians(-90),Math.toRadians(180));

            robot.wait(1000,robot.odometers);

            robot.changeAccuracy(1,Math.toRadians(1));

            robot.goToPos(49,-15,Math.toRadians(-90),0);

            shooterAngle.setPosition(0.33);
            pixScraper.setPosition(.7);

            robot.changeSpeed(1,1);

            //move past bar
            robot.goToPos(50,80,Math.toRadians(-90),0);

            finger.setPower(0);
            intake.setPower(0);

            robot.duelServoController(.57,shoulderL,shoulderR);

            wrist.setPosition(.8);

            robot.changeSpeed(.7,.7);

            robot.changeAccuracy(1, Math.toRadians(1));

            robot.goToPos(34,88,Math.toRadians(-90),Math.toRadians(90));

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 500) {

                slidesL.setTargetPosition(100);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(100);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.mecanumDrive(0,0,0,0.3);

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(250, robot.odometers);

            finger.setPower(0);

            robot.wait(500, robot.odometers);

            robot.changeAccuracy(.25, Math.toRadians(1));
            robot.goToPos(28,88,Math.toRadians(-90),Math.toRadians(90));

            ElapsedTime driveH = new ElapsedTime();

            //drive to the backdrop
            while(driveH.milliseconds() < 500) {

                slidesL.setTargetPosition(400);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(400);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.mecanumDrive(0,0,0,0.3);

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(750, robot.odometers);

            finger.setPower(0);

            robot.wait(500, robot.odometers);

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.wait(500, robot.odometers);

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

        }
        else if(startingPos == 3){
            camera.stopStreaming();
            robot.changeSpeed(1,1);
            //starting right

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(42,-9,0,0);


            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            robot.changeSpeed(.6,.6);

            robot.goToPos(50,-10,0,0);

            robot.changeAccuracy(.15,Math.toRadians(1));

            robot.goToPos(49.5,-16.5,Math.toRadians(-90),Math.toRadians(180));

            robot.wait(5000,robot.odometers);

            robot.changeAccuracy(1,Math.toRadians(1));

            pixScraper.setPosition(.62);

            robot.wait(500,robot.odometers);

            robot.changeSpeed(1,1);

            robot.goToPos(49,-5,Math.toRadians(-90),0);

            robot.duelServoController(.05,shoulderL,shoulderR);

            pixScraper.setPosition(.75);

            robot.changeSpeed(.4,.4);

            intake.setPower(.85);
            finger.setPower(1);

            robot.changeAccuracy(.25,Math.toRadians(1));

            //collecting pixel
            robot.goToPos(50,-18.25,Math.toRadians(-90),Math.toRadians(180));

            robot.wait(1000,robot.odometers);

            robot.changeAccuracy(1,Math.toRadians(1));

            robot.goToPos(49,-15,Math.toRadians(-90),0);

            shooterAngle.setPosition(0.33);
            pixScraper.setPosition(.7);

            robot.changeSpeed(1,1);

            //move past bar
            robot.goToPos(50,80,Math.toRadians(-90),0);

            finger.setPower(0);
            intake.setPower(0);

            robot.duelServoController(.57,shoulderL,shoulderR);

            wrist.setPosition(.8);

            robot.changeSpeed(.7,.7);

            robot.changeAccuracy(1, Math.toRadians(1));

            robot.goToPos(23,88,Math.toRadians(-90),Math.toRadians(90));

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 500) {

                slidesL.setTargetPosition(100);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(100);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.mecanumDrive(0,0,0,0.3);

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(250, robot.odometers);

            finger.setPower(0);

            robot.wait(500, robot.odometers);

            robot.changeAccuracy(.25, Math.toRadians(1));
            robot.goToPos(36,88,Math.toRadians(-90),Math.toRadians(-90));

            ElapsedTime driveH = new ElapsedTime();

            //drive to the backdrop
            while(driveH.milliseconds() < 500) {

                slidesL.setTargetPosition(400);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(400);
                slidesR.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.mecanumDrive(0.5,0,0,0.3);
                robot.refresh(robot.odometers);

            }

            robot.mecanumDrive(0,0,0,0.3);

            robot.wait(500, robot.odometers);

            //open
            finger.setPower(-1);

            robot.wait(750, robot.odometers);

            finger.setPower(0);

            robot.wait(500, robot.odometers);

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.wait(500, robot.odometers);

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