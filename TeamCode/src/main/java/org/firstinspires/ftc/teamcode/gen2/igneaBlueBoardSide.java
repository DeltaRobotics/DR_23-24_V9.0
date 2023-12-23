package org.firstinspires.ftc.teamcode.gen2;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="igneaBlueBoardSide")
//@Disabled

public class igneaBlueBoardSide extends LinearOpMode{

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

    public Servo finger = null;
    public Servo elbow = null;
    public Servo wrist = null;
    public Servo shoulder = null;
    public Servo intakeServo = null;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime servoTime = new ElapsedTime();

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        robotHardware robot = new robotHardware(hardwareMap);

        intake = hardwareMap.dcMotor.get("intake");

        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");

        finger = hardwareMap.servo.get("finger");
        elbow = hardwareMap.servo.get("elbow");
        wrist = hardwareMap.servo.get("wrist");
        shoulder = hardwareMap.servo.get("shoulder");
        intakeServo = hardwareMap.servo.get("intakeServo");

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

            //shoulder.setPosition(.08);
            //wrist.setPosition(.96);
            //elbow.setPosition(.77);
            finger.setPosition(.26);
            intakeServo.setPosition(0.78);

            //telemetry.addData("x", myPipeline.getRectMidpointX());
            //telemetry.addData("y", myPipeline.getRectMidpointY());
            telemetry.update();
        }
        camera.stopStreaming();

        //TODO adjust speeds in 2 and 3 like in 1
        if(startingPos == 1){
            camera.stopStreaming();
            //starting left
            //first drive forward
            robot.changeSpeed(.4,.4);

            shoulder.setPosition(.08);
            wrist.setPosition(.96);
            elbow.setPosition(.77);

            robot.goToPos(28,17,Math.toRadians(-90),0);

            intakeServo.setPosition(.56);
            robot.wait(1000,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(500,robot.odometers);
            intake.setPower(0);

            //move to board
            robot.goToPos(28,20,Math.toRadians(-90),0);

            shoulder.setPosition(.57);
            intakeServo.setPosition(0.78);

            robot.changeSpeed(.6,.6);

            //find pixel pos
            robot.goToPos(18,20,Math.toRadians(-90),Math.toRadians(90));

            wrist.setPosition(.52);

            //move to place
            robot.changeSpeed(.4,.4);
            robot.goToPos(18, 33, Math.toRadians(-90), 0);

            robot.wait(500, robot.odometers);

            //open
            finger.setPosition(.5);

            robot.wait(500, robot.odometers);

            wrist.setPosition(.46);
            shoulder.setPosition(.08);

            robot.wait(200, robot.odometers);

            wrist.setPosition(.96);

            robot.wait(1000, robot.odometers);

            robot.changeSpeed(.6,.6);

            robot.goToPos(5, 30, Math.toRadians(-90), Math.toRadians(90));

            robot.mecanumDrive(0,0,0,0);
        }
        else if(startingPos == 2){
            camera.stopStreaming();
            //starting middle
            //first drive forward
            robot.changeSpeed(.4,.4);

            shoulder.setPosition(.08);
            wrist.setPosition(.96);
            elbow.setPosition(.77);

            robot.goToPos(38,5,Math.toRadians(-90),0);

            intakeServo.setPosition(.56);
            robot.wait(1000,robot.odometers);
            intake.setPower(-0.05);
            robot.wait(500,robot.odometers);
            intake.setPower(0);

            //move to board
            robot.goToPos(38,20,Math.toRadians(-90),0);

            shoulder.setPosition(.57);
            intakeServo.setPosition(0.78);

            //find pixel pos
            robot.goToPos(25,20,Math.toRadians(-90),Math.toRadians(90));

            wrist.setPosition(.52);

            //move to place

            robot.goToPos(25, 33, Math.toRadians(-90), 0);

            robot.wait(500, robot.odometers);

            //open
            finger.setPosition(.5);

            robot.wait(500, robot.odometers);

            robot.goToPos(25, 28, Math.toRadians(-90), 0);

            robot.wait(500, robot.odometers);

            shoulder.setPosition(.08);
            wrist.setPosition(.96);

            robot.wait(500, robot.odometers);

            robot.goToPos(5, 30, Math.toRadians(-90), Math.toRadians(90));

            robot.mecanumDrive(0,0,0,0);
        }
        else if(startingPos == 3){
            camera.stopStreaming();
            //starting right
            robot.changeSpeed(.4,.4);
            //place purple pixel

            shoulder.setPosition(.08);
            wrist.setPosition(.96);
            elbow.setPosition(.77);

            robot.goToPos(28,0,Math.toRadians(-90),0);

            robot.goToPos(28,-5,Math.toRadians(-90),0);

            intakeServo.setPosition(.56);
            robot.wait(1000,robot.odometers);
            intake.setPower(-0.05);
            robot.wait(500,robot.odometers);
            intake.setPower(0);

            //go to board
            robot.goToPos(28,10,Math.toRadians(-90),0);

            shoulder.setPosition(.57);
            intakeServo.setPosition(0.78);

            //find pixel pos
            robot.goToPos(30,20,Math.toRadians(-90),Math.toRadians(90));

            wrist.setPosition(.52);

            //move to place
            robot.goToPos(30, 33, Math.toRadians(-90), 0);

            robot.wait(500, robot.odometers);

            //open
            finger.setPosition(.5);

            robot.wait(500, robot.odometers);

            wrist.setPosition(.46);
            shoulder.setPosition(.08);

            robot.wait(200, robot.odometers);

            wrist.setPosition(.96);

            robot.wait(1000, robot.odometers);

            robot.goToPos(5, 30, Math.toRadians(-90), Math.toRadians(90));

            robot.mecanumDrive(0,0,0,0);


        }

    }
}