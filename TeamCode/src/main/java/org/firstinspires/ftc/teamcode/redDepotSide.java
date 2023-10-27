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

@Autonomous(name="redDepotSide")
//@Disabled

public class redDepotSide extends LinearOpMode{

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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

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
        FtcDashboard.getInstance().startCameraStream(camera, 10);

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
            //first drive forward
            robot.goToPos(16,0,0,0);

            //nudging pixel mess
            robot.changeSpeed(.25,.25);
            robot.goToPos(13,0,0,0);
            robot.goToPos(16,-3,Math.toRadians(30),Math.toRadians(30));
            robot.goToPos(16,0,0,Math.toRadians(-90));
            robot.changeSpeed(.5,.5);

            //move back
            robot.goToPos(10,0,0,0);

            //move back to wall
            robot.goToPos(0,0,0,0);
        }
        else if(startingPos == 2){
            //starting middle
            //first drive forward
            robot.goToPos(24,0,Math.toRadians(10),0);

            //back up
            robot.goToPos(22,6,0,0);

            //back to Wall
            robot.goToPos(0,6,0,Math.toRadians(180));
        }
        else if(startingPos == 3){
            //starting right
            //place purple pixel
            robot.goToPos(18,0,0,0);
            robot.goToPos(12,0,0,0);
            robot.goToPos(12,18,0,Math.toRadians(90));
            robot.goToPos(24,18,0,0);
            robot.goToPos(24,-3,0,Math.toRadians(-90));

            robot.goToPos(25,5,0,0);

            robot.goToPos(0,5,0,0);
        }
    }
}