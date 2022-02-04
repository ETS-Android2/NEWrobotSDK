package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "test")
public class test extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor lift = hardwareMap.dcMotor.get("lift_dcMotor");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        //bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double driveSpeed = 1;
        double r;
        double gpAngle;
        double spin;
        double v1, v2, v3, v4;

        double fcgpAngle, robotAngle, finalAngle;
        double p1, p2, p3, p4;

        final int liftHome = 0;

        boolean fieldCentric = false;
        double angle = 0;
        double rightX;

        Orientation angles;
        BNO055IMU imu;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        telemetry.addData("Status", "Initalized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                if (driveSpeed == 1) { //if the current increment is 1, it'll switch to 0.5
                    driveSpeed = 0.5;
                    telemetry.addData("Slow Mode", "ON");
                } else { //if the current increment is not 1, it'll switch to 1
                    driveSpeed = 1;
                    telemetry.addData("Normal speed", "ON");
                }
            }
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            robotAngle = angles.firstAngle;

            r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            fcgpAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI/4;
            rightX = -gamepad1.right_stick_x;
            finalAngle = fcgpAngle - robotAngle;

            telemetry.addData("gpangle", fcgpAngle);
            telemetry.addData("finalAngle", finalAngle);

            p1 = driveSpeed * (((Math.cos(finalAngle)) * r) + rightX);
            p2 = driveSpeed * (((Math.sin(finalAngle)) * r) - rightX);
            p3 = driveSpeed * (((Math.sin(finalAngle)) * r) + rightX);
            p4 = driveSpeed * (((Math.cos(finalAngle)) * r) - rightX);

            telemetry.addData("p1", p1);
            telemetry.addData("p2", p2);
            telemetry.addData("p3", p3);
            telemetry.addData("p4", p4);



            fl.setPower(p1);
            fr.setPower(p2);
            bl.setPower(p3);
            br.setPower(p4);


            telemetry.update();


        }
    }
}