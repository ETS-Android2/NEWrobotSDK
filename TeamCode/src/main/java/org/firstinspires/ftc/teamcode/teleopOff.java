package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//TELEOP WITH 1 CONTROLLER
@TeleOp(name = "Tele w/ 1")
public class teleopOff extends LinearOpMode {
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

        CRServo spinServo = hardwareMap.crservo.get("crServo");
        Servo clawServo = hardwareMap.servo.get("clawServo");

        double driveSpeed = 1;
        double servoSpinSpeed = 0;
        final int liftHome = 0;
        double robotAngle = 0;

        boolean fieldCentric = false;
        double finalAngle;

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

            if(gamepad1.y){
                if(fieldCentric){
                    fieldCentric = false;
                }
                else{
                    fieldCentric = true;
                }
                telemetry.addData("fieldcentric", fieldCentric);
                telemetry.update();
                //sleep(1000);
            }

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            robotAngle = angles.firstAngle;

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y); //finds hypotenuse (power of each motor)
            double gpAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4; //finds angle of robot subtracted by pi/4 bc
            //it "shifts" the powers to each motor CW
            double rightX = -gamepad1.right_stick_x; //for rotating w/ right stick

            if(fieldCentric){
                finalAngle = gpAngle - robotAngle;
            }
            else{
                finalAngle = gpAngle;
            }

            final double v1 = driveSpeed * (r * Math.cos(finalAngle) + rightX);
            final double v2 = driveSpeed * (r * Math.sin(finalAngle) - rightX);
            final double v3 = driveSpeed * (r * Math.sin(finalAngle) + rightX);
            final double v4 = driveSpeed * (r * Math.cos(finalAngle) - rightX);
            /*
                r is mulitplier for power
                math.cos is used for fl and br bc fl&br are used to go diagonal top right, if you want to go faster to the right apply more power to
                those motors so closer joystick is to x axis faster robot go to that direction
                math.sin is used for same reason as ^ but to go faster forward/backwards
             */

            fl.setPower(v1);
            fr.setPower(v2);
            bl.setPower(v3);
            br.setPower(v4);

            if (gamepad1.right_bumper) {
                servoSpinSpeed = 1;

            }
            else if (gamepad1.left_bumper) {
                servoSpinSpeed = -1;

            }
            else if (gamepad1.right_bumper && gamepad1.left_bumper) {
                servoSpinSpeed = 0;
            }
            else {
                servoSpinSpeed = 0;

            }

            //spin carousel servo gamepad
            spinServo.setPower(servoSpinSpeed);

            if(gamepad1.dpad_down){
                lift.setTargetPosition(liftHome);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(.5);
                while(lift.isBusy()){

                }
                lift.setPower(0);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if((gamepad1.right_trigger > 0) && gamepad1.left_trigger == 0){
                lift.setPower(-gamepad1.right_trigger);
            }
            if((gamepad1.left_trigger > 0) && gamepad1.right_trigger == 0){
                lift.setPower(gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                lift.setPower(0);
            }


            if (gamepad1.x) {
                clawServo.setPosition(5);
            }
            if (gamepad1.b) {
                clawServo.setPosition(0);
            }

            telemetry.addData("Drive Speed Multiplier", driveSpeed);
            telemetry.addData("Field-Centric", fieldCentric);
            telemetry.update();
        }
    }
}
