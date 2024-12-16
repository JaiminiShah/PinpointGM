package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config

@TeleOp(name="GM_Telop2024", group="GreenMachine")  // Dares the name of the class and the group it is in.

public class GM_Telop2024 extends OpMode {

    final double ARM_TICKS_PER_DEGREE =
            28 *(250047.0/4913.0)*(100.0/20.0)*1/360.0;
         // ARM_TICKS_PER_DEGREE is 19.794

// 28.7 IS ARM_TICKS_PER_DEGREE
// Positions for the arms
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double GROUND_POS                = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double LOW_BASKET                = 90 * ARM_TICKS_PER_DEGREE;
    final double HIGH_BASKET               = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;
    //double armPosition = 0;
    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 300 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;




    final double MAX_ARM_POS = 1050;



    double  // Declares all double variables and their values
            speedVariable = .8;
    int speedVariable1=0;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double liftPosition = LIFT_COLLAPSED;
    double armLiftComp = 0;



    private final double zeroOffset = 90;

  /*  private PIDFController pidController=null;
    public static  double kp=0.77;//0.77;
    public static  double ki=0.003;//0.003;
    public static  double kd=0.004;//0.004;

    public static double kf=0.03;//0.03;*/

    public static int targetMotorPosition=0;

    //public static int targetDeg=0;
   // private final double ticksPerDegree=1425 / 360;
   // public  Encoder armMotorEncoder;
   // DcMotorEx armMotor = null;

    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize. If you're new to the stream Calc is short for calculator; I'm just using slang.
     */
    DcMotorEx
            rearLeft = null,
            rearRight = null,
            frontLeft = null,
            frontRight = null,
            armRotator = null,
            armRotator2 = null,
            armSlide = null;

    // servo names/declarations
    ServoImplEx
            wrist = null;

    // Continuous rotation servo
    CRServoImplEx
            intake = null;


    HardwareMap hwMap = null;
    ElapsedTime runTime = new ElapsedTime();

    @Override
    public void init() { //initialization class to be used at start of tele-op

        //PIDF implementation
        /*pidController = new PIDFController(kp, ki,kd, kf);
        pidController.setPIDF(kp,ki,kd, kf);
        int armPos =  armRotator.getCurrentPosition();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        double pid= pidController.calculate(armPos, targetMotorPosition);
// double feedforward = Math.cos(Math.toRadians(targetMotorPosition / ticksPerDegree)) * kf;
        double feedforward = Math.sin(Math.toRadians(armPos / ticksPerDegree + zeroOffset )) * kf;
        double armPositionInDeg = armPos/ticksPerDegree + zeroOffset;
        double power = pid * feedforward;
        armRotator.setPower(power);
        armRotator2.setPower(power);*/

        telemetry.addData("current pos", armPosition);
       // telemetry.addData("Arm position in degrees ", armPositionInDeg);
        //telemetry.addData("target", targetDeg);

        telemetry.update();

        // Motor Names
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
        armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        //liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        // Servo Names
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        intake = hardwareMap.get(CRServoImplEx.class, "intake");

        //Direction of motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armRotator2.setDirection(DcMotor.Direction.FORWARD);
        armSlide.setDirection(DcMotor.Direction.REVERSE);

        // Sets up motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        armRotator.setPower(0);
        armRotator.setTargetPosition(0);
        armRotator2.setPower(0);
        armRotator2.setTargetPosition(0);
        armSlide.setTargetPosition(0);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setPower(0);
        intake.setPower(0);


        // Sets position for servos
        wrist.setPosition(0);

        // Hi thanks for reading this comments I worked really hard on them :)
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

    /*
     * Code will run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * This code will run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
        armRotator.setTargetPosition(0);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator.setPower(0);
        armRotator2.setTargetPosition(0);
        armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator2.setPower(0);

        armSlide.setTargetPosition(0);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetrymotorprint();
        telemetryarm();
        telemetrylift();

    }

    /*
     * Code will run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {


        //==========================================================\\
        //                        GamePad One                       \\
        //==========================================================\\

       armPositionFudgeFactor=FUDGE_FACTOR*(gamepad1.right_trigger +(-gamepad1.left_trigger));
        // armRotator manual lifting mostly for testing or as a last resort
      /*  if (gamepad2.a){
            /* This is the intaking/collecting arm position*/
      /*      armPosition=GROUND_POS;
        } else if (gamepad2.b) {
            armPosition=ARM_CLEAR_BARRIER;
        } else if (gamepad2.x) {
            armPosition=HIGH_BASKET;
        }else if (gamepad2.y){
            armPosition=LOW_BASKET;
        }else {
            armPosition=ARM_COLLAPSED_INTO_ROBOT;
        }*/
      /*  if (armPosition < 45 * ARM_TICKS_PER_DEGREE){
            armLiftComp = (0.25568 * liftPosition);
        }
        else{
            armLiftComp = 0;
        }
        armRotator.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
        armRotator2.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
        ((DcMotorEx) armRotator).setVelocity(2100);
        ((DcMotorEx) armRotator2).setVelocity(2100);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/


        // armRotator manual lifting mostly for testing or as a last resort
      /*  if(gamepad1.right_trigger>.1){
            armRotator.setPower(.25);
            armRotator2.setPower(.25);
        }

        else if(gamepad1.left_trigger>.1){
            armRotator.setPower(-.25);
            armRotator2.setPower(-.25);
        }*/


        // Drive Train controls
        float FLspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        float BLspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        float FRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x;
        float BRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x;

        rearLeft.setPower(Range.clip((-BLspeed * speedVariable), -1, 1));
        rearRight.setPower(Range.clip((BRspeed * speedVariable), -1, 1));
        frontLeft.setPower(Range.clip((FLspeed * speedVariable), -1, 1));
        frontRight.setPower(Range.clip((-FRspeed * speedVariable), -1, 1));


        // DriveTrain Speed Controls
        if (gamepad1.dpad_left) speedVariable -= 0.1;
        if (gamepad1.dpad_right) speedVariable += 0.1;

        speedVariable = Range.clip(speedVariable, 0, 1);



        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
         /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

        //==========================================================\\
        //                        GamePad Two                       \\
        //==========================================================\\

        // Controls for intake
        if (gamepad2.left_bumper) {
            intake.setPower(0.5);
        }

        else if (gamepad2.right_bumper) {
            intake.setPower(-0.5);
        }

        // Controls for arm slide
        if (gamepad2.right_trigger > .1) {
            liftPosition += 2800 * cycletime;
        }

        else if (gamepad2.left_trigger > .1) {
            liftPosition -= 2800 * cycletime;
        }

        // Makes sure the lift does not go beyond parameters
        if (liftPosition > MAX_ARM_POS){
            liftPosition = MAX_ARM_POS;
        }

        else if (liftPosition < 0){
            liftPosition = 0;
        }

        armSlide.setTargetPosition((int) (liftPosition));

        ((DcMotorEx) armSlide).setVelocity(200);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Controls for armRotator
        if (gamepad2.a) {
            armPosition = GROUND_POS;
            wrist.setPosition(.3);
        }

        else if (gamepad2.b) {
            armPosition = LOW_BASKET;
            wrist.setPosition(.5);
        }

        else if (gamepad2.y) {
            armPosition = HIGH_BASKET;
            wrist.setPosition(.5);

        }
        if (armPosition < 45 * ARM_TICKS_PER_DEGREE){
            armLiftComp = (0.25568 * liftPosition);
        }
        else{
            armLiftComp = 0;
        }
        armRotator.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
        armRotator2.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
        ((DcMotorEx) armRotator).setVelocity(2100);
        ((DcMotorEx) armRotator2).setVelocity(2100);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);





        looptime = getRuntime();
        cycletime = looptime-oldtime;
        oldtime = looptime;

        //==========================================================\\
        //                        Telemetry                         \\
        //==========================================================\\

        telemetrymotorprint();
        //telemetrylift();
        telemetryarm();


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

        // this will send a telemetry message to signify robot waiting
        telemetry.addLine("I 'm Ready");
        telemetry.update();

    }
    public void telemetrylift(){
        telemetry.addData("lift variable", MAX_ARM_POS);
        telemetry.addData("Lift Target Position",armSlide.getTargetPosition());
        telemetry.addData("lift current position", armSlide.getCurrentPosition());
        telemetry.addData("liftMotor Current:",(armSlide.getCurrent(CurrentUnit.AMPS)));
        telemetry.update();

    }
    public void telemetryarm(){
        telemetry.addData("Rotator Target Position: ", armRotator.getTargetPosition());
        telemetry.addData("Rotator2 target Position",armRotator2.getTargetPosition());
        telemetry.addData("Rotator Target Position: ",armRotator.getTargetPosition());
        telemetry.addData("Rotator2 Target Position:",armRotator2.getTargetPosition());
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
    }
    // pixelArm.setTargetPosition(pixelArm.getCurrentPosition());

}