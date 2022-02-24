//THIS ONE STARTS CLOSER TO TEAM STATION (DUCK)
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import java.security.KeyStore;

@Autonomous(name="basic blue w/ cara", group="Basic Autos") //telling robot it is autonoumous
public class basicBlue4Auto extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL / ENCODERTICKS) * GEARRATIO;


    Orientation angles;

    BNO055IMU imu;
    Robot robot = new Robot();

    public void runOpMode() {

        robot.init(hardwareMap, telemetry);
        robot.clawClamp();


        waitForStart();
        if (opModeIsActive()) {
            robot.driveForwardDistance(.25, (int) (610/TICKSTOMMTRAVEL));
            robot.rotate(55);
            robot.liftMotor(-1940, -1);
            robot.driveForwardDistance(.25, (int) (220/TICKSTOMMTRAVEL));
            robot.clawOpen();
            sleep(500);
            robot.driveBackDistance(.25, (int) (150/TICKSTOMMTRAVEL));
            robot.liftMotor(1940, .5);
            sleep(500);
            robot.driveBackDistance(.25, (int) (70/TICKSTOMMTRAVEL));
            robot.rotate(0);
            robot.driveBackDistance(.5, (int) (610/TICKSTOMMTRAVEL));
            robot.driveForwardDistance(.5, (int) (70/TICKSTOMMTRAVEL));
            robot.rotate(90);
            robot.driveBackDistance(.5, (int) (650/TICKSTOMMTRAVEL));
            robot.rotate(25);
            robot.duckMotor(1, 500);
            robot.driveForwardDistance(.5, (int) (30/TICKSTOMMTRAVEL));
            robot.rotate(0);
            robot.driveForwardDistance(.5, (int) (320/TICKSTOMMTRAVEL));
            sleep(500);
            robot.rotate(0);
        }
    }
}