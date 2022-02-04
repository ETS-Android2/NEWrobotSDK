package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


//TELEOP WITH 1 CONTROLLER
@TeleOp(name = "Motor test")
public class motorTest extends LinearOpMode {
    public void runOpMode() {

        DcMotor lift = hardwareMap.dcMotor.get("lift_dcMotor");

        waitForStart();
        while (opModeIsActive()) {
            lift.setPower(-gamepad1.right_trigger);
            lift.setPower(gamepad1.left_trigger);
            while (gamepad1.a) {
                lift.setPower(-1);
            }
            lift.setPower(0);


            telemetry.addData("right trigger", gamepad1.right_trigger);
            telemetry.addData("left trigger", gamepad1.left_trigger);
            telemetry.addData("motor", ((DcMotorEx)lift).getVelocity());
            telemetry.update();
        }
    }
}

