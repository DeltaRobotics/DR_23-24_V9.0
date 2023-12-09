package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PoleDetectionPipeline;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="blueBoardSideV2")
@Disabled

public class blueBoardSideV2 extends LinearOpMode{

    private OpenCvCamera camera;//find webcam statement
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double Inches_PER_METER = 39.37;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.0508;

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

    AprilTagDetection tagOfInterest = null;

    //non-wheels
    public Servo launcher = null;
    public DcMotor slidesL = null;
    public DcMotor slidesR = null;
    public Servo clawL = null;
    public Servo clawR = null;
    public Servo wrist = null;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime servoTime = new ElapsedTime();

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

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

        // OpenCV lift webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //april tag detection pipline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //OpenCV PoleDetectionPipeline
        PoleDetectionPipeline myPipeline;
        camera.setPipeline(myPipeline = new PoleDetectionPipeline());
        // Configuration of PoleDetectionPipeline
        myPipeline.ConfigurePipeline(75, 75,50,125,  CAMERA_WIDTH, CAMERA_HEIGHT);
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
            } else if(myPipeline.getRectMidpointX() >= 430 && myPipeline.getRectMidpointX() <= 525){
                telemetry.addData("location", 1);
                startingPos = 1;
            } else if(myPipeline.getRectMidpointX() >= 75 && myPipeline.getRectMidpointX() <= 160){
                telemetry.addData("location", 3);
                startingPos = 3;
            } else{
                telemetry.addData("location", "no head seen");
                startingPos = 1;
            }

            slidesR.setTargetPosition(slideEncoder);
            slidesR.setPower(.5);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slidesL.setTargetPosition(slideEncoder);
            slidesL.setPower(.5);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (slidesL.getCurrentPosition() > 300 && !pixelGrab){
                wrist.setPosition(.4);
                doubleLoop = servoTime.seconds();
                pixelGrab = true;
            }
            if (doubleLoop + .75 < servoTime.seconds()){
                clawR.setPosition(1);
                clawL.setPosition(0);
            }
            if (doubleLoop + 2 < servoTime.seconds()){
                wrist.setPosition(.55);
            }
            if (doubleLoop + 3 < servoTime.seconds()){
                slideEncoder = 50;
            }

            //telemetry.addData("x", myPipeline.getRectMidpointX());
            //telemetry.addData("y", myPipeline.getRectMidpointY());
            telemetry.update();
        }

        camera.setPipeline(aprilTagDetectionPipeline);

        if(startingPos == 1){
            //starting left
            //first drive forward
            robot.changeSpeed(.3,.3);
            robot.goToPos(16,0,0,0);

            //nudging pixel mess
            robot.changeSpeed(.25,.25);
            robot.goToPos(10,0,0,0);
            robot.goToPos(16,-3,Math.toRadians(30),Math.toRadians(30));
            robot.goToPos(16,0,0,Math.toRadians(-90));

            robot.changeSpeed(.3,.3);

            //move back
            robot.goToPos(10,0,0,0);


            //turn and move to backdrop
            robot.goToPos(10,20,Math.toRadians(-90),0);

            x = 16;
            y = 35;
            finalAngle = Math.toRadians(-90);
            slideEncoder = 2000;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, 0);

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 1250) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            robot.wait(250, robot.odometers);

            clawL.setPosition(.1);
            clawR.setPosition(.7);

            robot.wait(500, robot.odometers);

            //back away
            robot.goToPos(13,30,Math.toRadians(-90),0);

            //move to wall
            x = 1;
            y = 30;
            finalAngle = Math.toRadians(-90);
            slideEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            /* placing
            robot.changeAccuracy(0.25,Math.toRadians(0.5));

            //back up to bar
            robot.goToPos(3,5,0,Math.toRadians(180));

            ElapsedTime driveR = new ElapsedTime();

            while(driveR.milliseconds() < 400) {

                robot.mecanumDrive(0.5,0,0,0.75);
                robot.refresh(robot.odometers);

            }

            robot.goToPos(3,-5,0,Math.toRadians(-90));

            //back up under bar
            robot.changeSpeed(0.35,0.35);
            robot.goToPos(3,-26.75,0,Math.toRadians(-90));
            robot.changeSpeed(0.5,0.5);

            //drive through bar
            robot.goToPos(23,-26.75,0,0);
            robot.goToPos(47,-26.75,0,0);

            //turn under bar
            robot.goToPos(50,-26.75,Math.toRadians(90),0);

            //move towards pile
            x = 48;
            y = -80;
            finalAngle = Math.toRadians(90);
            sildeEncoder = 600;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, 0);

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.wrist.setPosition(.55);
            }

            robot.clawR.setPosition(1);
            robot.clawL.setPosition(0);

            robot.wait(250,robot.odometers);

            robot.wrist.setPosition(.4);
            */




            ElapsedTime driveR = new ElapsedTime();

            //drive to wall
            while(driveR.milliseconds() < 1000) {

                robot.mecanumDrive(0,0.75,0,0.75);
                robot.refresh(robot.odometers);

            }

            ElapsedTime driveF2 = new ElapsedTime();

            //drive to park
            while(driveF2.milliseconds() < 1000) {

                robot.mecanumDrive(-0.75,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            slideEncoder = 0;
            while(slidesL.getCurrentPosition() > 10 && slidesR.getCurrentPosition() > 10){
                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //While loop for goToPosSingle:
            //Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy

            robot.mecanumDrive(0,0,0,0);
        }
        else if(startingPos == 2){
            //starting middle
            //first drive forward
            robot.changeSpeed(.4,.4);
            robot.goToPos(26,0,Math.toRadians(10),0);

            //back up
            robot.goToPos(22,6,0,0);

            x = 24;
            y = 35;
            finalAngle = Math.toRadians(-90);
            slideEncoder = 2000;

            //drive to the backdrop
            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, 0);

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 1250) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            slideEncoder = 1500;

            //move slides down
            while(slidesL.getCurrentPosition() > 1500){

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.refresh(robot.odometers);
            }

            robot.wait(1000, robot.odometers);

            clawL.setPosition(.1);
            clawR.setPosition(.7);

            robot.wait(1000, robot.odometers);

            //back away
            robot.goToPos(20,30,Math.toRadians(-90),0);

            //move to wall
            x = 1;
            y = 30;
            finalAngle = Math.toRadians(-90);
            slideEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            ElapsedTime driveR = new ElapsedTime();

            //drive to wall
            while(driveR.milliseconds() < 1000) {

                robot.mecanumDrive(0,0.75,0,0.75);
                robot.refresh(robot.odometers);

            }

            ElapsedTime driveF2 = new ElapsedTime();

            //drive to park
            while(driveF2.milliseconds() < 1000) {

                robot.mecanumDrive(-0.75,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            slideEncoder = 0;
            while(slidesL.getCurrentPosition() > 10 && slidesR.getCurrentPosition() > 10){
                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //While loop for goToPosSingle:
            //Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy

            robot.mecanumDrive(0,0,0,0);
        }
        else if(startingPos == 3){
            //starting right
            robot.changeSpeed(.4,.4);
            //place purple pixel
            robot.goToPos(18,0,0,0);
            //todo speed up and reduce the accuracy for the parts the robot is not touching the pixel
            robot.goToPos(6,0,0,Math.toRadians(180));
            robot.goToPos(6,20,0,0);
            robot.goToPos(24,20,0,0);
            robot.goToPos(24,-4,0,Math.toRadians(-90));

            robot.goToPos(25,5,0,0);

            x = 29;
            y = 35;
            finalAngle = Math.toRadians(-90);
            slideEncoder = 2000;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, 0);

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 1000) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            clawL.setPosition(.1);
            clawR.setPosition(.7);

            robot.wait(500, robot.odometers);

            //back away
            robot.goToPos(13,30,Math.toRadians(-90),0);

            //move to wall
            x = 1;
            y = 30;
            finalAngle = Math.toRadians(-90);
            slideEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            ElapsedTime driveR = new ElapsedTime();

            //drive to wall
            while(driveR.milliseconds() < 1000) {

                robot.mecanumDrive(0,0.75,0,0.75);
                robot.refresh(robot.odometers);

            }

            ElapsedTime driveF2 = new ElapsedTime();

            //drive to park
            while(driveF2.milliseconds() < 1000) {

                robot.mecanumDrive(-0.75,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            slideEncoder = 0;
            while(slidesL.getCurrentPosition() > 10 && slidesR.getCurrentPosition() > 10){
                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //While loop for goToPosSingle:
            //Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy

            robot.mecanumDrive(0,0,0,0);


        }

    }
    public double findTagLocation(int interest){
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if(currentDetections.size() != 0)
        {

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == interest)
                {
                    tagOfInterest = tag;
                    break;
                }
            }
        }
        return tagOfInterest.pose.x*Inches_PER_METER;
    }
}
