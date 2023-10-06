package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="driveWithOdo")
@Disabled

public class driveWithOdo extends LinearOpMode{

    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        robot.launcher.setPosition(1);


        int armSetPos = 0;

        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 0.75);

            //if (gamepad1.x){
            //    //zero position
            //    robot.wrist.setPosition(0);
            //    armSetPos = 0;
            //}
            //if (gamepad1.b){
            //    //placing
            //    armSetPos = 50;
            //}
            //if (gamepad1.a){
            //    //collecting
            //    armSetPos = 100;
            //}
//
            ////Runs the arm using the pid
            //robot.arm.setTargetPosition(armSetPos);
            //robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.arm.setPower(robot.odoPID(armSetPos,robot.arm.getCurrentPosition()));
//
            //if(gamepad1.right_bumper){
            //    //grab
            //    robot.finger.setPosition(0.5);
            //}
            //if(gamepad1.left_bumper){
            //    //release
            //    robot.finger.setPosition(0);
            //}
//
            //if(gamepad1.dpad_up){
            //    //slight change
            //    robot.wrist.setPosition(robot.wrist.getPosition() + .05);
            //}
            //if(gamepad1.dpad_down){
            //    robot.wrist.setPosition(robot.wrist.getPosition() - .05);
            //}
//
            //robot.refresh(robot.odometers);
//
            //if(gamepad2.a && gamepad2.b){
            //    robot.launcher.setPosition(0);
            //}
//
            ////keep for testing
            //if(gamepad2.x){
            //    robot.launcher.setPosition(1);
            //}
            //if(gamepad2.y){
            //    robot.launcher.setPosition(0);
            //}
//
            //telemetry.addData("motor Power",robot.arm.getPower());
            //telemetry.addData("arm encoder", robot.arm.getCurrentPosition());
            //telemetry.addData("servo", robot.launcher.getPosition());
            //telemetry.update();
        }
    }
}