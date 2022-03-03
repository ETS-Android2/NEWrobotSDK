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

@Autonomous(name="red white wh park only", group="Basic Autos") //telling robot it is autonoumous
public class basicRed1Auto extends LinearOpMode {

    //four rotations for lift all up
    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    Robot robot = new Robot();

    public void runOpMode(){

        robot.init(hardwareMap, telemetry);
        // robot.clawClamp();

        waitForStart();
        if(opModeIsActive()){
            robot.liftMotor(-454, -.5);
            robot.driveRightDistance(.5, 2000);
            robot.liftMotor(454, .5);
        }
    }

}
