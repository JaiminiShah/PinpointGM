package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="GM_Teleop12024", group="GreenMachine")  // Dares the name of the class and the group it is in.

public class GM_Teleop12024 extends OpMode {
    DcMotorEx

        rearLeft = null,
            rearRight = null,
    frontLeft = null,
    frontRight = null,
    armRotator = null,
    armRotator2 = null,
    armSlide = null;
    final double liftIncrement = 23.0;
    final double MAX_ARM_POS = 2000;
    // servo names/declarations
    Servo
            wrist = null;

    // Continuous rotation servo
    CRServo
            intake = null;
    double liftPosition=0;
    double hangPosition = 0;
    //Control hub IMU declaration
    IMU imu;

    HardwareMap hwMap = null;
    ElapsedTime runTime = new ElapsedTime();
    double armPower,armPower1;
    double kp=0.77
            ,ki=0.003
            ,kd=0.004;
    double GROUND_POS=0;
    double ARM_CLEAR_BARRIER=50;
    double LOW_BASKET=280;
    double HIGH_BASKET=320;
    double armPosition;
    PIDController pid1=new PIDController(kp,ki,kd);
    double  // Declares all double variables and their values
            speedVariable = .8;
    int speedVariable1=0;

    @Override
    public void init() {
        telemetry.update();

        // Motor Names
        rearLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        rearRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
        armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        // Servo Names
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");

        //Direction of motors
        //Direction of motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armRotator2.setDirection(DcMotor.Direction.REVERSE);
        armSlide.setDirection(DcMotor.Direction.REVERSE);

        // Sets up motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        armRotator.setPower(0);
        armRotator2.setPower(0);
        armSlide.setPower(0);
        intake.setPower(0);
        // Sets position for servos
        wrist.setPosition(.7);
        // setting all subsystem and motors to stop and reset encoder mode
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        //telemetrylift();
        telemetry.update();
        runTime.reset();

    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(backRightPower);

        // Controls for intake
        if (gamepad2.left_bumper) {
            intake.setPower(0.5);
        }

        else if (gamepad2.right_bumper) {
            intake.setPower(-0.5);
        }

        else {
            intake.setPower(0);
        }
        // Controls for arm slide (real)
        if (gamepad2.left_trigger > .1 && liftPosition > 0) {
            liftPosition-=liftIncrement;
        }

        if (gamepad2.right_trigger > .1 && liftPosition<2000) {
            liftPosition+=liftIncrement;
        }

        // Makes sure the lift does not go beyond parameters
        if (liftPosition > MAX_ARM_POS){
            liftPosition = MAX_ARM_POS;
        }

        else if (liftPosition < 0){
            liftPosition = 0;
        }
        // Moves slide to the position
        armSlide.setTargetPosition((int) (liftPosition));

        ((DcMotorEx) armSlide).setVelocity(400);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Controls for armRotator
        if (gamepad2.a) {

            armPower = pid1.update(LOW_BASKET,armRotator.getCurrentPosition(),15);
            armPower1=pid1.update(LOW_BASKET,armRotator.getCurrentPosition(),15);
            armPosition=LOW_BASKET;
           // wrist.setPosition(.7);
        }

        else if (gamepad2.b) {
            armPower = pid1.update(HIGH_BASKET,armRotator.getCurrentPosition(),15);
            armPower1=pid1.update(HIGH_BASKET,armRotator.getCurrentPosition(),15);
            armPosition=HIGH_BASKET;
           // armPosition = LOW_BASKET;
            //wrist.setPosition(.5);
        }

        else if (gamepad2.y) {
            armPower = pid1.update(ARM_CLEAR_BARRIER,armRotator.getCurrentPosition(),8);
            armPower1=pid1.update(ARM_CLEAR_BARRIER,armRotator.getCurrentPosition(),8);
           armPosition = ARM_CLEAR_BARRIER;
            //wrist.setPosition(.5);
        } else if (gamepad2.x) {
            armPower = pid1.update(GROUND_POS,armRotator.getCurrentPosition(),8);
            armPower1=pid1.update(GROUND_POS,armRotator.getCurrentPosition(),8);
            armPosition = GROUND_POS;
            //wrist.setPosition(.5);
        }


        armRotator.setTargetPosition((int)(armPosition));
         armRotator.setPower(armPower);
        //((DcMotorEx) armRotator).setVelocity(200);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armRotator2.setTargetPosition((int) (armPosition));
        armRotator2.setPower(armPower1);
       // ((DcMotorEx) armRotator2).setVelocity(200);
        armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

}
