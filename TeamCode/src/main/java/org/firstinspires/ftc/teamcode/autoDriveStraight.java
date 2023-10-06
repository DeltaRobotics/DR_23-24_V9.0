package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="autoDriveStraight")
//@Disabled
public class autoDriveStraight extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        //telemetry.addData("hello",1.5);
        telemetry.addData("heading", robot.GlobalHeading);
        telemetry.addData("distanceX", robot.GlobalX);
        telemetry.addData("value",0);
        telemetry.update();

        waitForStart();

        robot.wait(2000, robot.odometers);

        double y = 0;
        double x = 20;
        double finalAngle = Math.toRadians(0);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
            //robot.goToPosSingle(20, 00, 00, 0);
            //telemetry.addData("hello world",x);


            telemetry.addData("value",robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0)));
            //robot.goToPosSingle(x, y, finalAngle, 0);

            telemetry.addData("pid 90",robot.odoTurnPID(0,Math.toRadians(90)));

            telemetry.addData("globalY",robot.GlobalY);
            telemetry.addData("distanceX", robot.GlobalX);




            telemetry.addData("heading", robot.GlobalHeading);
            telemetry.addData("90", 2);
            telemetry.update();
        }
        telemetry.addData("bye world", 1);
        telemetry.update();

        //telemetry.addData("deg",robot.GlobalHeading * 57.295);
        //telemetry.update();

        //robot.wait(2000, robot.odometers);

    }
}
