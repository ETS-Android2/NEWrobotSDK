package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "basic red w/ cara", group = "Basic Autos")
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
            robot.clawClamp();
            sleep(100);
            robot.driveForwardDistance(.4, (int) (610/TICKSTOMMTRAVEL));
            robot.rotate(-45);
            robot.liftMotor(100, -1);
            robot.driveForwardDistance(.4, (int) (220/TICKSTOMMTRAVEL));
            robot.clawOpen();
            sleep(500);
            robot.driveBackDistance(.4, (int) (150/TICKSTOMMTRAVEL));
            //robot.liftMotor(500, 1);
            robot.driveBackDistance(.4, (int) (70/TICKSTOMMTRAVEL));
            robot.rotate(0);
            robot.driveBackDistance(.5, (int) (610/TICKSTOMMTRAVEL));
            robot.driveForwardDistance(.5, (int) (220/TICKSTOMMTRAVEL));
            robot.rotate(-90);
            robot.driveBackDistance(.5, (int) (620/TICKSTOMMTRAVEL));
            robot.rotate(-25);
            robot.driveBackDistance(.5, (int) (250/TICKSTOMMTRAVEL));
            robot.duckMotor(1, -500);
            robot.driveForwardDistance(.5, (int) (30/TICKSTOMMTRAVEL));
            robot.rotate(0);
            robot.driveForwardDistance(.5, (int) (250/TICKSTOMMTRAVEL));
            robot.rotate(0);
        }
    }
}
