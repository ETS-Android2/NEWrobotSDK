//THIS ONE STARTS CLOSE TO ALLIANCE WAREHOUSE
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

@Autonomous(name="basic Blue w/ wh", group="Basic Autos") //telling robot it is autonoumous
public class basicBlue3Auto extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    //lvl 1 -454 ticks, lvl 2 -1130 ticks, lvl 3 -1940
    Robot robot = new Robot();

    public void runOpMode(){

        robot.init(hardwareMap, telemetry);
        robot.clawClamp();

        waitForStart();
        if(opModeIsActive()){
            robot.driveForwardDistance(.25, (int) (610/TICKSTOMMTRAVEL));
            robot.rotate(-45);
            robot.liftMotor(-454, -.5);
            robot.driveForwardDistance(.25, (int) (220/TICKSTOMMTRAVEL)); //random number
            robot.clawOpen();
            sleep(500);
            robot.driveBackDistance(.25, (int) (150/TICKSTOMMTRAVEL));    //away from hub and to warehouse
            //robot.liftMotor(500, 1);
            robot.rotate(0);
            robot.driveBackDistance(.6, (int) (610/TICKSTOMMTRAVEL)); //go to white warehouse //go to white warehouse
            robot.driveLeftDistance(.5, 1000);
        }
    }

}
