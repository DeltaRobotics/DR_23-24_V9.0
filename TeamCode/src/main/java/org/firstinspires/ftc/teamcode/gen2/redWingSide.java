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
    public Servo intakeServo = null;
    public Servo shoulderL = null;
    public Servo shoulderR = null;
    public Servo shooterAngle = null;

    public DcMotor slidesL = null;
    public DcMotor slidesR = null;
    public Servo wrist = null;
    public CRServo finger = null;

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
        shooterAngle = hardwareMap.servo.get("shooterAngle");

        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");
        finger = hardwareMap.crservo.get("finger");
        wrist = hardwareMap.servo.get("wrist");

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

            robot.duelServoController(.04,shoulderL,shoulderR);
            wrist.setPosition(.34);
            shooterAngle.setPosition(0);
            telemetry.update();
        }

        if(startingPos == 1){
            //starting left
            camera.stopStreaming();
            //first drive forward
            robot.changeSpeed(.4,.4);
            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(43,9,0,0);

            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            robot.goToPos(50,8,Math.toRadians(90),0);

            robot.wait(10000,robot.odometers);

            shooterAngle.setPosition(0.33);

            robot.changeSpeed(.6,.6);

            robot.goToPos(50,-80,Math.toRadians(90),0);

            robot.duelServoController(.57,shoulderL,shoulderR);

            robot.wait(500,robot.odometers);

            //placing on board
            robot.goToPos(31,-80,Math.toRadians(90),Math.toRadians(-90));

            wrist.setPosition(.8);

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 3000) {

                slidesL.setTargetPosition(500);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(500);
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
            while(driveG.milliseconds() < 1500) {

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
            //starting middle
            camera.stopStreaming();
            robot.changeSpeed(.4,.4);

            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(47,0,0,0);

            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            robot.goToPos(50,0,0,0);

            robot.wait(500,robot.odometers);

            robot.goToPos(50,-10,Math.toRadians(90),0);

            shooterAngle.setPosition(0.33);

            robot.wait(10000,robot.odometers);

            robot.changeSpeed(.6,.6);

            robot.goToPos(50,-80,Math.toRadians(90),0);

            robot.duelServoController(.57,shoulderL,shoulderR);

            robot.goToPos(25,-80,Math.toRadians(90),Math.toRadians(-90));

            robot.wait(500,robot.odometers);

            robot.changeSpeed(.4,.4);

            wrist.setPosition(.8);

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 3000) {

                slidesL.setTargetPosition(500);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(500);
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
            while(driveG.milliseconds() < 1500) {

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
            //starting right
            camera.stopStreaming();
            //place purple pixel
            robot.changeSpeed(.4,.4);
            robot.duelServoController(.1,shoulderL,shoulderR);
            wrist.setPosition(.34);

            robot.goToPos(28,0,Math.toRadians(-90),0);

            robot.goToPos(28,-8,Math.toRadians(-90),Math.toRadians(180));

            robot.goToPos(28,-4,Math.toRadians(-90),0);

            robot.wait(100,robot.odometers);
            intake.setPower(-0.1);
            robot.wait(100,robot.odometers);
            intake.setPower(0);

            intakeServo.setPosition(.56);
            robot.wait(500,robot.odometers);

            robot.goToPos(28,10,Math.toRadians(-90),0);

            robot.wait(1000,robot.odometers);

            robot.goToPos(50,10,Math.toRadians(90),0);

            robot.wait(5000,robot.odometers);

            shooterAngle.setPosition(0.33);

            robot.changeSpeed(.6,.6);

            robot.goToPos(50,-80,Math.toRadians(90),0);

            robot.duelServoController(.57,shoulderL,shoulderR);

            robot.goToPos(20,-80,Math.toRadians(90),Math.toRadians(-90));

            robot.wait(500,robot.odometers);

            robot.changeSpeed(.4,.4);

            wrist.setPosition(.8);

            ElapsedTime driveF = new ElapsedTime();

            //drive to the backdrop
            while(driveF.milliseconds() < 3000) {

                slidesL.setTargetPosition(500);
                slidesL.setPower(1);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesR.setTargetPosition(500);
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
            while(driveG.milliseconds() < 1500) {

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
}