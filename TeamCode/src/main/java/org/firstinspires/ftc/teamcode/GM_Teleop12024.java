package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import android.util.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="GM_Teleop2024New", group="GreenMachine")  // Dares the name of the class and the group it is in.

public class GM_Teleop12024 extends OpMode {

    
    DcMotorEx
            rearLeft = null,
            rearRight = null,
            frontLeft = null,
            frontRight = null,
            armRotator = null,
            armRotator2 = null,
            armSlide = null;

    final double armIncrement = 1;

    final double liftIncrement = 20;
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
            ,kd=0.004
            ,f=0.03;
    double GROUND_POS=50;
    double ARM_CLEAR_BARRIER=200;
    double LOW_BASKET=530;
    double HIGH_BASKET=500;
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
        wrist.setPosition(0);

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
        float FLspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        float BLspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        float FRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x;
        float BRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x;

        rearLeft.setPower(Range.clip((-BLspeed * speedVariable), -1, 1));
        rearRight.setPower(Range.clip((BRspeed * speedVariable), -1, 1));
        frontLeft.setPower(Range.clip((FLspeed * speedVariable), -1, 1));
        frontRight.setPower(Range.clip((-FRspeed * speedVariable), -1, 1));


        if(gamepad1.left_trigger > .1){
            armPosition-=armIncrement;

        }

        if(gamepad1.right_trigger > .1)  {
            armPosition+=armIncrement;

        }


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

        ((DcMotorEx) armSlide).setVelocity(700);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Controls for armRotator
        if (gamepad2.b) {

            armPower = pid1.update(LOW_BASKET,armRotator.getCurrentPosition(),15);
            armPower1=pid1.update(LOW_BASKET,armRotator.getCurrentPosition(),15);
            armPosition=LOW_BASKET;
            wrist.setPosition(.5);
        }

        else if (gamepad2.y) {
            armPower = pid1.update(HIGH_BASKET,armRotator.getCurrentPosition(),15);
            armPower1=pid1.update(HIGH_BASKET,armRotator.getCurrentPosition(),15);
            armPosition=HIGH_BASKET;
            wrist.setPosition(.5);
        }

        else if (gamepad2.x) {
            armPower = pid1.update(ARM_CLEAR_BARRIER,armRotator.getCurrentPosition(),8);
            armPower1=pid1.update(ARM_CLEAR_BARRIER,armRotator.getCurrentPosition(),8);
            armPosition = ARM_CLEAR_BARRIER;
            wrist.setPosition(.7);

        }

        else if (gamepad2.a) {
            armPower = pid1.update(GROUND_POS,armRotator.getCurrentPosition(),8);
            armPower1=pid1.update(GROUND_POS,armRotator.getCurrentPosition(),8);
            armPosition = GROUND_POS;
            wrist.setPosition(.3);
        }

        armRotator.setTargetPosition((int)(armPosition));
        armRotator.setPower(armPower);
        //((DcMotorEx) armRotator).setVelocity(200);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armRotator2.setTargetPosition((int) (armPosition));
        armRotator2.setPower(armPower1);
        // ((DcMotorEx) armRotator2).setVelocity(200);
        armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (gamepad2.dpad_right){
            wrist.setPosition(.2);
        }

        if (gamepad2.dpad_left){
            wrist.setPosition(.3);
        }


    }

    public void telemetrymotorprint(){
        telemetry.clear();
        telemetry.addData("Drive Train Speed: " , speedVariable);
        telemetry.addData("BRMotor2", "Position : %2d, Power : %.2f", rearRight.getCurrentPosition(), rearRight.getPower());
        telemetry.addData("FRMotor2", "Position : %2d, Power : %.2f", frontRight.getCurrentPosition(), frontRight.getPower());

        telemetry.addData("FLMotor2", "Position : %2d, Power : %.2f", frontLeft.getCurrentPosition(), frontLeft.getPower());
        telemetry.addData("BLMotor2", "Position : %2d, Power : %.2f", rearLeft.getCurrentPosition(), rearLeft.getPower());
        telemetry.addLine("left joystick | ")
                .addData("x", gamepad1.left_stick_x)
                .addData("y", gamepad1.left_stick_y);
        telemetry.addLine("right joystick | ")
                .addData("x", gamepad1.right_stick_x)
                .addData("y", gamepad1.right_stick_y);

        telemetry.addData("Arm Position: " , armPosition);
        // this will send a telemetry message to signify robot waiting
        telemetry.addLine("I 'm Ready");
        telemetry.update();

    }



    //Code will run ONCE after the driver hits STOP
    @Override
    public void stop() {
        // Sets all motors to zero power except Arms to keep pos
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
        armSlide.setPower(0);
        armRotator.setPower(0);
        armRotator2.setPower(0);
        //      hanger.setPower(0);
    }
}



