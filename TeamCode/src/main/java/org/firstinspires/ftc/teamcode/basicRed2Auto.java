package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Blue white wh park only", group = "Basic Autos")
public class basicRed2Auto extends LinearOpMode {
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;
    CRServo crServo = null;

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    Orientation angles;

    BNO055IMU imu;
    Robot robot = new Robot();

    public void runOpMode(){

        robot.init(hardwareMap, telemetry);
        //robot.clawClamp();

        waitForStart();
        if(opModeIsActive()){
            robot.liftMotor(-454, -.5);
            robot.driveLeftDistance(.5, 2000);
            robot.liftMotor(454, .5);
        }
    }
}
