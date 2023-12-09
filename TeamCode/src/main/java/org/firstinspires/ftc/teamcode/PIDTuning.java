package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="PIDTuning")
@Disabled

public class PIDTuning extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        telemetry.addData("GlobalHeading", robot.GlobalHeading);
        telemetry.addData("finalAngle",0);
        telemetry.update();

        waitForStart();

        robot.changeSpeed(0.4,0.4);

        double y = 0;
        double x = 50;
        double finalAngle = 0;

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);


            telemetry.addData("GlobalHeading", robot.GlobalHeading);
            telemetry.addData("x",robot.GlobalX);
            telemetry.addData("y",robot.GlobalY);
            telemetry.update();
        }
        robot.mecanumDrive(0,0,0,0);
        telemetry.addData("bye world", 1);
        telemetry.addData("GlobalHeading", robot.GlobalHeading);
        telemetry.addData("x",robot.GlobalX);
        telemetry.addData("y",robot.GlobalY);
        telemetry.update();

        //telemetry.addData("deg",robot.GlobalHeading * 57.295);
        //telemetry.update();

        robot.wait(10000, robot.odometers);
        telemetry.addData("GlobalHeading", robot.GlobalHeading);
        telemetry.addData("x",robot.GlobalX);
        telemetry.addData("y",robot.GlobalY);
        telemetry.update();

    }
}
