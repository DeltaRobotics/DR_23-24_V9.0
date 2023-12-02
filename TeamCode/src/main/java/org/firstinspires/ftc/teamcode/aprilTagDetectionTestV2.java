package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Camera.PoleDetectionPipeline;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="aprilTagDetectionTestV2")

public class aprilTagDetectionTestV2 extends LinearOpMode
{
    //IMU object
    BNO055IMU imu;

    //motor/servo objects
    public DcMotorEx SlideRight1 = null;
    public DcMotorEx SlideLeft1 = null;
    public DcMotorEx SlideRight2 = null;
    public DcMotorEx SlideLeft2 = null;
    public Servo Claw0 = null;
    public Servo Claw1 = null;

    //april tag camera
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double Inches_PER_METER = 39.37;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 1430;
    double fy = 1430;
    double cx = 480;
    double cy = 620;
    // UNITS ARE METERS
    double tagsize = 0.166;
    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;

    private static final int CAMERA_WIDTH  = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution
    // Yellow Range                                      Y      Cr     Cb
    public static Scalar YellowScalarLowerYCrCb = new Scalar(  0.0, 100, 0);
    public static Scalar YellowScalarUpperYCrCb = new Scalar(255.0, 250.0, 100);
    // Yellow Range                                      Y      Cr     Cb
    public static Scalar BlueScalarLowerYCrCb = new Scalar(  0.0, 0, 170);
    public static Scalar BlueScalarUpperYCrCb = new Scalar(255.0, 100.0, 255);
    // Yellow Range                                      Y      Cr     Cb
    public static Scalar RedScalarLowerYCrCb = new Scalar(  0.0, 180, 90);
    public static Scalar RedScalarUpperYCrCb = new Scalar(255.0, 255.0, 170);
    int allianceColor = 0;

    //i2c sensors
    public NormalizedColorSensor color = null;
    public VoltageSensor ControlHub_VoltageSensor;
    //todo add distance sensors



    @Override
    public void runOpMode() throws InterruptedException {



        // OpenCV lift webcam init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        //april tag camera init
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        telemetry.addData("sleeve cam inited", 0);
        telemetry.update();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.addData("sleeve cam running", 0);
        telemetry.update();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 1 || tag.id == 2 || tag.id == 3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }


            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            //stop aprilTag camera


            telemetry.addData("sleeve cam off", 0);
            telemetry.update();


            /* Update the telemetry */
            if(tagOfInterest != null)
            {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }
            else
            {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Actually do something useful */
            if(tagOfInterest == null || tagOfInterest.id == 1){
                //trajectory 1 dot
            }
            else if(tagOfInterest.id == 2){
                //trajectory 2 dots
            }
            else if(tagOfInterest.id == 3){
                //trajectory 3 dots
            }
        }

        camera.stopStreaming();


    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f inches", detection.pose.x*Inches_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f inches", detection.pose.y*Inches_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f inches", detection.pose.z*Inches_PER_METER));
        //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
