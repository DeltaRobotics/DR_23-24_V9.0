package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PoleDetectionPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="redBoardSide")
//@Disabled

public class redBoardSide extends LinearOpMode{

    private OpenCvCamera camera;//find webcam statement

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Yellow Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 230.0, 50.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 180.0);

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
        //OpenCV PoleDetectionPipeline
        PoleDetectionPipeline myPipeline;
        camera.setPipeline(myPipeline = new PoleDetectionPipeline());
        // Configuration of PoleDetectionPipeline
        myPipeline.ConfigurePipeline(75, 75,50,125,  CAMERA_WIDTH, CAMERA_HEIGHT);
        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);


        telemetry.addData("Pipeline status", "ready");

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

        while(!isStarted() && !isStopRequested()){
            if(myPipeline.getRectMidpointX() >= 220 && myPipeline.getRectMidpointX() <= 420){
                telemetry.addData("location", 2);
                startingPos = 5;
            } else if(myPipeline.getRectMidpointX() >= 440 && myPipeline.getRectMidpointX() <= 600){
                telemetry.addData("location", 1);
                startingPos = 4;
            } else if(myPipeline.getRectMidpointX() >= 80 && myPipeline.getRectMidpointX() <= 200){
                telemetry.addData("location", 3);
                startingPos = 6;
            } else{
                telemetry.addData("location", "no head seen");
                startingPos = 4;
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
        camera.stopStreaming();
        if(startingPos == 4){
            camera.stopStreaming();
            robot.changeSpeed(.4,.4);
            //starting left

            //place purple pixel
            robot.goToPos(18,0,0,0);
            robot.goToPos(6,0,0,Math.toRadians(180));
            robot.goToPos(6,-20,0,0);
            robot.goToPos(24,-20,0,0);
            robot.goToPos(24,5,0,Math.toRadians(90));

            robot.goToPos(24,-5,0,0);

            //turn and move to backdrop
            robot.goToPos(30,-25,Math.toRadians(90),0);

            x = 32;
            y = -30;
            finalAngle = Math.toRadians(90);
            slideEncoder = 2000;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 1500) {

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
            robot.goToPos(33,-30,Math.toRadians(90), Math.toRadians(180));


            //move to wall
            x = 3;
            y = -30;
            finalAngle = Math.toRadians(90);
            slideEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            ElapsedTime driveR = new ElapsedTime();

            //drive to wall
            while(driveR.milliseconds() < 500) {

                robot.mecanumDrive(0,-0.75,0,0.75);
                robot.refresh(robot.odometers);

            }

            ElapsedTime driveF2 = new ElapsedTime();

            //drive to park
            while(driveF2.milliseconds() < 1500) {

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
        else if(startingPos == 5){
            //starting middle
            camera.stopStreaming();
            robot.changeSpeed(.4,.4);
            //first drive forward
            robot.goToPos(24,0,Math.toRadians(-25),0);

            //back up
            robot.goToPos(20,-5,0,0);


            //turn and move to backdrop
            robot.goToPos(24,-20,Math.toRadians(90),0);

            robot.changeAccuracy(0.5,1);
            x = 24;
            y = -30;
            finalAngle = Math.toRadians(90);
            slideEncoder = 1800;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, 0);

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            robot.changeAccuracy(1,1);

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 1500) {

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
            robot.goToPos(27,-30,Math.toRadians(90),Math.toRadians(180));


            //move to wall
            x = 3;
            y = -30;
            finalAngle = Math.toRadians(90);
            slideEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));

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

                robot.mecanumDrive(0,-0.75,0,0.75);
                robot.refresh(robot.odometers);

            }

            ElapsedTime driveF2 = new ElapsedTime();

            //drive to park
            while(driveF2.milliseconds() < 1500) {

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
        else if(startingPos == 6){
            //starting right
            camera.stopStreaming();
            robot.changeSpeed(.4,.4);
            //first drive forward
            robot.goToPos(16,0,0,0);

            //nudging pixel mess
            robot.changeSpeed(.25,.25);
            robot.goToPos(9,0,0,0);
            robot.goToPos(17,2,Math.toRadians(-30),Math.toRadians(-30));
            robot.goToPos(17,1,0,Math.toRadians(90));
            robot.changeSpeed(.4,.4);

            //move back
            robot.goToPos(10,1,0,0);

            //turn and move to backdrop
            robot.goToPos(10,-25,Math.toRadians(90),Math.toRadians(-90));

            x = 20;
            y = -30;
            finalAngle = Math.toRadians(90);
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
            while(driveF.milliseconds() < 1500) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            slideEncoder = 1500;

            while(slidesL.getCurrentPosition() > 1500){

                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.refresh(robot.odometers);
            }

            robot.wait(500, robot.odometers);

            clawL.setPosition(.1);
            clawR.setPosition(.7);

            robot.wait(500, robot.odometers);


            //back away
            robot.goToPos(18,-30,Math.toRadians(90),0);


            //move to wall
            x = 3;
            y = -30;
            finalAngle = Math.toRadians(90);
            slideEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));

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

                robot.mecanumDrive(0,-0.75,0,0.75);
                robot.refresh(robot.odometers);

            }

            ElapsedTime driveF2 = new ElapsedTime();

            //drive to park
            while(driveF2.milliseconds() < 1500) {

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
}