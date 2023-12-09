package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;

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

    String[] arrow = {">","-","-","-","-"};

    Boolean[] cursorB = {false, false, false, false};
    int[] cursorI = {0};

    int[] boolItems = {0,1,2,4};
    int[] intItems = {3};


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(arrow[0] + " )Item 0 ", cursorB[0]);
        telemetry.addData(arrow[1] + " )Item 1 ", cursorB[1]);
        telemetry.addData(arrow[2] + " )Item 2 ", cursorB[2]);
        telemetry.addData(arrow[3] + " )Item 3 ", cursorI[0]);
        telemetry.addData(arrow[4] + " )Item 4 ", cursorB[3]);
        telemetry.update();

        ElapsedTime servoTime = new ElapsedTime();

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //robotHardware robot = new robotHardware(hardwareMap);
        //robot.resetDriveEncoders();

        while(!isStarted() && !isStopRequested()){
            telemetry.update();

            //select number
            if(gamepad1.dpad_down && !upPressed && cursor < arrow.length-1){
                arrow[cursor] = "-";
                cursor++;
                arrow[cursor] = ">";
                upPressed = true;
            }
            if(gamepad1.dpad_up && !downPressed && cursor > 0){
                arrow[cursor] = "-";
                cursor--;
                arrow[cursor] = ">";
                downPressed = true;
            }
            if(!gamepad1.dpad_down && upPressed){
                upPressed = false;
            }
            if(!gamepad1.dpad_up && downPressed){
                downPressed = false;
            }


            if(gamepad1.a && !aPressed){
                //set true
                for(int i = 0; i == cursorB.length -1; i++){
                    int check = boolItems[i];
                    if(check == cursor){
                        cursorB[cursor] = true;
                    }
                }

                aPressed = true;
            }
            if(gamepad1.b && !bPressed){
                //set false
                cursorB[cursor] = false;

                bPressed = true;
            }
            //single button
            if (!gamepad1.a && aPressed){
                aPressed = false;
            }
            if(!gamepad1.b && bPressed){
                bPressed = false;
            }


            telemetry.addData(arrow[0] + " )Item 0 ", cursorB[0]);
            telemetry.addData(arrow[1] + " )Item 1 ", cursorB[1]);
            telemetry.addData(arrow[2] + " )Item 2 ", cursorB[2]);
            telemetry.addData(arrow[3] + " )Item 3 ", cursorI[0]);
            telemetry.addData(arrow[4] + " )Item 4 ", cursorB[3]);
            telemetry.update();
        }
    }
}