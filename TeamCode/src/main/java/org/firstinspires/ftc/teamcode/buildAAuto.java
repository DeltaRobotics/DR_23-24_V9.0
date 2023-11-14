package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PoleDetectionPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="buildAAuto")
@Disabled

public class buildAAuto extends LinearOpMode{

    private OpenCvCamera camera;//find webcam statement

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Yellow Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb;
    public static Scalar scalarUpperYCrCb;

    //blue
    //public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 105.0, 150.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 200.0, 255.0);

    //red
    //public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 230.0, 50.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 180.0);

    double poleOffset = 0;
    double poleOffsetPower = 0;

    int startingPos;

    double doubleLoop = 10;
    boolean pixelGrab = false;

    int sildeEncoder = 400;

    int x;
    int y;
    double finalAngle;

    int cursor = 0;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean upPressed = false;
    boolean downPressed = false;

    String[] arrow = {">","-"};

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(arrow[0] + " )Item 0 ", false);
        telemetry.addData(arrow[1] + ")Item 1 ", false);
        telemetry.addData(arrow[1] + ")Item 1 ", arrow.length);

        ElapsedTime servoTime = new ElapsedTime();

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //robotHardware robot = new robotHardware(hardwareMap);
        //robot.resetDriveEncoders();

        while(!gamepad1.start){
            telemetry.update();

            //select number
            if(gamepad1.dpad_up && !upPressed && cursor < arrow.length-1){
                arrow[cursor] = "-";
                cursor++;
                arrow[cursor] = ">";
                upPressed = true;
            }
            if(gamepad1.dpad_down && !downPressed && cursor > 0){
                arrow[cursor] = "-";
                cursor--;
                arrow[cursor] = ">";
                downPressed = true;
            }
            if(!gamepad1.dpad_up && upPressed){
                upPressed = false;
            }
            if(!gamepad1.dpad_down && downPressed){
                downPressed = false;
            }


            if(gamepad1.a){
                if(cursor == 0){
                    telemetry.addData(arrow[0] + " )Item 0 ", true);
                }
                if(cursor == 1){
                    telemetry.addData(arrow[1] + ")Item 1 ", true);
                }

            }
            if(gamepad1.b){
                //set false
                if(cursor == 0){
                    telemetry.addData(arrow[0] + " )Item 0 ", false);
                }
                if(cursor == 1){
                    telemetry.addData(arrow[1] + ")Item 1 ", false);
                }
            }

            if(!gamepad1.a && aPressed){
                aPressed = false;
            }
            if(!gamepad1.b && bPressed){
                bPressed = false;
            }
        }
    }
}