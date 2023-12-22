package org.firstinspires.ftc.teamcode.gen2;

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
import org.firstinspires.ftc.teamcode.robotHardware;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="redWingSide")
//@Disabled

public class redWingSide extends LinearOpMode{

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
    public DcMotor intake = null;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime servoTime = new ElapsedTime();

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        robotHardware robot = new robotHardware(hardwareMap);

        intake = hardwareMap.dcMotor.get("intake");

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

        while(!isStarted() && !isStopRequested()){
            if(myPipeline.getRectMidpointX() >= 170 && myPipeline.getRectMidpointX() <= 390){
                telemetry.addData("location", 2);
                startingPos = 2;
            } else if(myPipeline.getRectMidpointX() >= 400 && myPipeline.getRectMidpointX() <= 625){
                telemetry.addData("location", 1);
                startingPos = 1;
            } else if(myPipeline.getRectMidpointX() >= 0 && myPipeline.getRectMidpointX() <= 110){
                telemetry.addData("location", 3);
                startingPos = 3;
            } else{
                telemetry.addData("location", "no head seen");
                startingPos = 3;
            }

            //telemetry.addData("x", myPipeline.getRectMidpointX());
            //telemetry.addData("y", myPipeline.getRectMidpointY());
            telemetry.update();
        }

        if(startingPos == 1){
            //starting left
            camera.stopStreaming();
            //first drive forward
            robot.changeSpeed(.2,.4);
            robot.changeAccuracy(0.1,1);

            robot.goToPos(43,8,0,0);

            intake.setPower(-0.05);
            robot.wait(500,robot.odometers);
            intake.setPower(0);

            robot.goToPos(47,8,0,0);

            /**robot.goToPos(16,0,0,0);

            //nudging pixel mess
            robot.changeSpeed(.25,.25);
            robot.goToPos(12,0,0,0);
            robot.goToPos(17,-3,Math.toRadians(30),Math.toRadians(30));
            robot.goToPos(17,0,0,Math.toRadians(-90));
            robot.changeSpeed(.5,.5);

            //move back
            robot.goToPos(10,0,0,0);

            slideEncoder = 0;
            while(slidesL.getCurrentPosition() > 10 && slidesR.getCurrentPosition() > 10){
                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
             **/
        }
        else if(startingPos == 2){
            //starting middle
            camera.stopStreaming();
            robot.changeSpeed(.2,.4);
            robot.changeAccuracy(0.1,1);

            robot.goToPos(45,0,0,0);

            intake.setPower(-0.05);
            robot.wait(500,robot.odometers);
            intake.setPower(0);

            robot.goToPos(49,0,0,0);

            /**robot.goToPos(24,0,Math.toRadians(10),0);

            //back up
            robot.goToPos(22,6,0,0);

            slideEncoder = 0;
            while(slidesL.getCurrentPosition() > 10 && slidesR.getCurrentPosition() > 10){
                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
             **/
        }
        else if(startingPos == 3){
            //starting right
            camera.stopStreaming();
            //place purple pixel
            robot.changeSpeed(.4,.4);

            robot.goToPos(28,0,Math.toRadians(-90),0);

            robot.goToPos(28,-3,Math.toRadians(-90),0);

            intake.setPower(-0.05);
            robot.wait(500,robot.odometers);
            intake.setPower(0);

            robot.goToPos(28,10,Math.toRadians(-90),0);

            /**robot.goToPos(18,0,0,0);
            robot.goToPos(12,0,0,0);
            robot.goToPos(12,18,0,Math.toRadians(90));
            robot.goToPos(24,18,0,0);
            robot.goToPos(24,-3,0,Math.toRadians(-90));

            robot.goToPos(25,5,0,0);

            slideEncoder = 0;
            while(slidesL.getCurrentPosition() > 10 && slidesR.getCurrentPosition() > 10){
                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
             **/
        }
    }
}