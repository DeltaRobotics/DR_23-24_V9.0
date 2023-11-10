package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
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
                startingPos = 2;
            } else if(myPipeline.getRectMidpointX() >= 440 && myPipeline.getRectMidpointX() <= 600){
                telemetry.addData("location", 1);
                startingPos = 1;
            } else if(myPipeline.getRectMidpointX() >= 80 && myPipeline.getRectMidpointX() <= 200){
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
            camera.stopStreaming();
            //starting left

            //place purple pixel
            robot.goToPos(18,0,0,0);
            robot.goToPos(12,0,0,0);
            robot.goToPos(12,-20,0,Math.toRadians(-90));
            robot.goToPos(24,-20,0,0);
            robot.goToPos(24,4,0,Math.toRadians(90));

            robot.goToPos(25,-5,0,0);

            //turn and move to backdrop
            robot.goToPos(30,-20,Math.toRadians(90),0);

            x = 30;
            y = -25;
            finalAngle = Math.toRadians(90);
            sildeEncoder = 2000;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 1500) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            robot.clawL.setPosition(.1);
            robot.clawR.setPosition(.7);

            robot.wait(500, robot.odometers);


            //back away
            robot.goToPos(33,-30,Math.toRadians(90), Math.toRadians(180));


            //move to wall
            x = 3;
            y = -30;
            finalAngle = Math.toRadians(90);
            sildeEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            robot.goToPos(24,0,Math.toRadians(-25),0);

            //back up
            robot.goToPos(18,-5,0,0);


            //turn and move to backdrop
            robot.goToPos(29,-20,Math.toRadians(90),0);

            robot.changeAccuracy(0.5,1);
            x = 29;
            y = -34;
            finalAngle = Math.toRadians(90);
            sildeEncoder = 1800;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, 0);

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


            robot.changeAccuracy(1,1);

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 750) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            robot.wait(750, robot.odometers);

            robot.clawL.setPosition(.1);
            robot.clawR.setPosition(.7);

            robot.wait(750, robot.odometers);


            //back away
            robot.goToPos(27,-30,Math.toRadians(90),Math.toRadians(180));


            //move to wall
            x = 3;
            y = -30;
            finalAngle = Math.toRadians(90);
            sildeEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));

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

                robot.mecanumDrive(0,-0.75,0,0.75);
                robot.refresh(robot.odometers);

            }

            ElapsedTime driveF2 = new ElapsedTime();

            //drive to park
            while(driveF2.milliseconds() < 1500) {

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
            //first drive forward
            robot.goToPos(17,0,0,0);

            //nudging pixel mess
            robot.changeSpeed(.25,.25);
            robot.goToPos(12,0,0,0);
            robot.goToPos(18,3,Math.toRadians(-30),Math.toRadians(-30));
            robot.goToPos(18,2,0,Math.toRadians(90));
            robot.changeSpeed(.5,.5);

            //move back
            robot.goToPos(10,1,0,0);

            //turn and move to backdrop
            robot.goToPos(10,-25,Math.toRadians(90),Math.toRadians(-90));

            x = 18;
            y = -25;
            finalAngle = Math.toRadians(90);
            sildeEncoder = 2000;

            robot.changeAccuracy(1,Math.toRadians(.5));

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));

                robot.slidesR.setTargetPosition(sildeEncoder);
                robot.slidesR.setPower(.5);
                robot.slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.slidesL.setTargetPosition(sildeEncoder);
                robot.slidesL.setPower(.5);
                robot.slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            robot.goToPos(18,-25,Math.toRadians(90),0);

            robot.changeAccuracy(1,Math.toRadians(1));


            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 1500) {

                robot.mecanumDrive(-0.5,0,0,0.5);
                robot.refresh(robot.odometers);

            }

            robot.wait(500, robot.odometers);

            robot.clawL.setPosition(.1);
            robot.clawR.setPosition(.7);

            robot.wait(500, robot.odometers);


            //back away
            robot.goToPos(18,-30,Math.toRadians(90),0);


            //move to wall
            x = 3;
            y = -30;
            finalAngle = Math.toRadians(90);
            sildeEncoder = 200;

            while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

                robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));

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

                robot.mecanumDrive(0,-0.75,0,0.75);
                robot.refresh(robot.odometers);

            }

            ElapsedTime driveF2 = new ElapsedTime();

            //drive to park
            while(driveF2.milliseconds() < 1500) {

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