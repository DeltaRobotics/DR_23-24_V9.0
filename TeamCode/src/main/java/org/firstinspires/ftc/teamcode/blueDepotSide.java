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

@Autonomous(name="blueDepotSide")
//@Disabled

public class blueDepotSide extends LinearOpMode{

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
            if(myPipeline.getRectMidpointX() >= 220 && myPipeline.getRectMidpointX() <= 400){
                telemetry.addData("location", 2);
                startingPos = 2;
            } else if(myPipeline.getRectMidpointX() >= 420 && myPipeline.getRectMidpointX() <= 600){
                telemetry.addData("location", 1);
                startingPos = 1;
            } else if(myPipeline.getRectMidpointX() >= 80 && myPipeline.getRectMidpointX() <= 210){
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
        if(startingPos == 1){
            //starting left
            camera.stopStreaming();
            //place purple pixel
            robot.goToPos(18,0,0,0);
            robot.goToPos(12,0,0,0);
            robot.goToPos(12,-18,0,Math.toRadians(-90));
            robot.goToPos(24,-18,0,0);
            robot.goToPos(24,4,0,Math.toRadians(90));

            robot.goToPos(25,2,0,0);

            slideEncoder = 0;
            while(slidesL.getCurrentPosition() > 10 && slidesR.getCurrentPosition() > 10){
                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else if(startingPos == 2){
            //starting middle
            camera.stopStreaming();
            //first drive forward
            robot.goToPos(24,0,Math.toRadians(-25),0);

            //back up
            robot.goToPos(22,0,0,0);

            slideEncoder = 0;
            while(slidesL.getCurrentPosition() > 10 && slidesR.getCurrentPosition() > 10){
                slidesR.setTargetPosition(slideEncoder);
                slidesR.setPower(.5);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesL.setTargetPosition(slideEncoder);
                slidesL.setPower(.5);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else if(startingPos == 3){
            //starting right
            camera.stopStreaming();
            //first drive forward
            robot.goToPos(16,0,0,0);

            //nudging pixel mess
            robot.changeSpeed(.25,.25);
            robot.goToPos(13,0,0,0);
            robot.goToPos(16,3,Math.toRadians(-30),Math.toRadians(-30));
            robot.goToPos(16,0,0,Math.toRadians(90));
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
        }

    }
}