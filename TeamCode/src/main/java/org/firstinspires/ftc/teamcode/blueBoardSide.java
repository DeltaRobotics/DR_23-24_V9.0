package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PoleDetectionPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="blueBoardSide")
//@Disabled

public class blueBoardSide extends LinearOpMode{

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

    int sildeEncoder = 400;

    int x;
    int y;
    double finalAngle;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime servoTime = new ElapsedTime();

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        robotHardware robot = new robotHardware(hardwareMap);

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
        FtcDashboard.getInstance().startCameraStream(camera, 10);

        //!isStarted() && !isStopRequested()
        //this replaces waitForStart()

        while(!isStarted() && !isStopRequested()){
            if(myPipeline.getRectMidpointX() >= 240 && myPipeline.getRectMidpointX() <= 370){
                telemetry.addData("location", 2);
                startingPos = 2;
            } else if(myPipeline.getRectMidpointX() >= 450 && myPipeline.getRectMidpointX() <= 525){
                telemetry.addData("location", 1);
                startingPos = 1;
            } else if(myPipeline.getRectMidpointX() >= 100 && myPipeline.getRectMidpointX() <= 230){
                telemetry.addData("location", 3);
                startingPos = 3;
            } else{
                telemetry.addData("location", "no head seen");
                startingPos = 1;
            }

            robot.slidesR.setTargetPosition(sildeEncoder);
            robot.slidesR.setPower(.5);
            robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.slidesL.setTargetPosition(sildeEncoder);
            robot.slidesL.setPower(.5);
            robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (robot.slidesL.getCurrentPosition() > 300 && !pixelGrab){
                robot.wrist.setPosition(.4);
                doubleLoop = servoTime.seconds();
                pixelGrab = true;
            }
            if (doubleLoop + .75 < servoTime.seconds()){
                robot.clawR.setPosition(1);
                robot.clawL.setPosition(0);
            }
            if (doubleLoop + 2 < servoTime.seconds()){
                robot.wrist.setPosition(.55);
            }

            //telemetry.addData("x", myPipeline.getRectMidpointX());
            //telemetry.addData("y", myPipeline.getRectMidpointY());
            telemetry.update();
        }
        if(startingPos == 1){
            //starting left
            camera.stopStreaming();
            //first drive forward
            robot.goToPos(16,0,0,0);

            //nudging pixel mess
            robot.changeSpeed(.25,.25);
            robot.goToPos(12,0,0,0);
            robot.goToPos(17,-3,Math.toRadians(30),Math.toRadians(30));
            robot.goToPos(17,0,0,Math.toRadians(-90));
            robot.changeSpeed(.5,.5);

            //move back
            robot.goToPos(10,0,0,0);

            //turn and move to backdrop
            robot.goToPos(10,20,Math.toRadians(-90),0);

            x = 12;
            y = 35;
            finalAngle = Math.toRadians(-90);
            sildeEncoder = 2000;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, 0);

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 500) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            robot.wait(250, robot.odometers);

            robot.clawL.setPosition(.1);
            robot.clawR.setPosition(.7);

            robot.wait(500, robot.odometers);

            //back away
            robot.goToPos(13,30,Math.toRadians(-90),0);

            //move to wall
            x = 1;
            y = 30;
            finalAngle = Math.toRadians(-90);
            sildeEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            //for cycling
            //back up to bar
            //robot.goToPos(3,5,0,Math.toRadians(180));

            //strafe into bar
            //robot.goToPos(3,-26,0,Math.toRadians(-90));


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

            sildeEncoder = 0;
            while(robot.slidesL.getCurrentPosition() > 10 && robot.slidesR.getCurrentPosition() > 10){
                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //While loop for goToPosSingle:
            //Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy

            robot.mecanumDrive(0,0,0,0);
        }
        else if(startingPos == 2){
            //starting middle
            camera.stopStreaming();
            //first drive forward
            robot.goToPos(26,0,Math.toRadians(10),0);

            //back up
            robot.goToPos(22,0,0,0);

            x = 20;
            y = 35;
            finalAngle = Math.toRadians(-90);
            sildeEncoder = 2000;

            //drive to the backdrop
            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, 0);

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 500) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            robot.clawL.setPosition(.1);
            robot.clawR.setPosition(.7);

            robot.wait(2000, robot.odometers);

            //back away
            robot.goToPos(20,30,Math.toRadians(-90),0);

            //move to wall
            x = 1;
            y = 30;
            finalAngle = Math.toRadians(-90);
            sildeEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

            sildeEncoder = 0;
            while(robot.slidesL.getCurrentPosition() > 10 && robot.slidesR.getCurrentPosition() > 10){
                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //While loop for goToPosSingle:
            //Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy

            robot.mecanumDrive(0,0,0,0);
        }
        else if(startingPos == 3){
            //starting right
            camera.stopStreaming();

            //place purple pixel
            robot.goToPos(18,0,0,0);
            //todo speed up and reduce the accuracy for the parts the robot is not touching the pixel
            robot.goToPos(12,0,0,0);
            robot.goToPos(12,20,0,Math.toRadians(90));
            robot.goToPos(24,20,0,0);
            robot.goToPos(24,-3,0,Math.toRadians(-90));

            robot.goToPos(25,5,0,0);

            x = 30;
            y = 30;
            finalAngle = Math.toRadians(-90);
            sildeEncoder = 2000;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, 0);

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 750) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            robot.clawL.setPosition(.1);
            robot.clawR.setPosition(.7);

            robot.wait(500, robot.odometers);

            //back away
            robot.goToPos(35,30,Math.toRadians(-90),Math.toRadians(180));

            //move to wall
            x = 3;
            y = 30;
            finalAngle = Math.toRadians(-90);
            sildeEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

            sildeEncoder = 0;
            while(robot.slidesL.getCurrentPosition() > 10 && robot.slidesR.getCurrentPosition() > 10){
                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //While loop for goToPosSingle:
            //Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy

            robot.mecanumDrive(0,0,0,0);


        }

    }
}